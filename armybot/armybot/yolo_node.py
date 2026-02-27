import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from od_msg.srv import SrvDepthPosition 
from cv_bridge import CvBridge
import cv2
import numpy as np

# YOLO 라이브러리 (ultralytics 예시)
from ultralytics import YOLO

class YoloVisionNode(Node):
    def __init__(self):
        super().__init__('yolo_vision_node')
        
        # 1. 학습된 YOLO 모델 로드 (IR 흑백 이미지로 학습된 가중치 파일)
        self.model = YOLO('/home/rokey/cobot2_ws/src/armybot/resource/brass_magazine.pt') 
        self.bridge = CvBridge()
        
        self.current_ir_img = None
        self.current_depth_img = None
        self.intrinsics = None  # 카메라 내부 파라미터 (초점거리, 광학중심 등)

        # 2. RealSense D435i 토픽 구독 (Left IR, Depth, CameraInfo)
        self.create_subscription(Image, '/camera/camera/infra1/image_rect_raw', self.ir_cb, 10)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_cb, 10)
        self.create_subscription(CameraInfo, '/camera/camera/infra1/camera_info', self.info_cb, 10)
        
        # 3. 로봇 미션 노드와 통신할 서비스 서버 오픈
        self.srv = self.create_service(SrvDepthPosition, '/get_3d_position', self.handle_get_position)
        
        self.get_logger().info("👁️ 흑백 IR 기반 YOLO 비전 노드가 준비되었습니다.")

    def ir_cb(self, msg):
        """실시간 IR(흑백) 이미지 업데이트"""
        try:
            ir_img = self.bridge.imgmsg_to_cv2(msg, "mono8") # 또는 "8UC1"
            self.current_ir_img = cv2.cvtColor(ir_img, cv2.COLOR_GRAY2BGR)
        except Exception as e:
            self.get_logger().error(f"IR 이미지 변환 실패: {e}")

    def depth_cb(self, msg):
        """실시간 뎁스 이미지 업데이트"""
        try:
            self.current_depth_img = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f"Depth 이미지 변환 실패: {e}")

    def info_cb(self, msg):
        """카메라 내부 파라미터(Intrinsics) 1회만 수신하여 저장"""
        if self.intrinsics is None:
            self.intrinsics = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'cx': msg.k[2],
                'cy': msg.k[5]
            }
            self.get_logger().info("✅ 카메라 파라미터 수신 완료!")

    def handle_get_position(self, request, response):
        target_name = request.target
        
        if self.current_ir_img is None or self.current_depth_img is None or self.intrinsics is None:
            self.get_logger().warn("⚠️ 카메라 데이터가 아직 준비되지 않았습니다.")
            return response
            
        self.get_logger().info(f"[{target_name}] 탐색 요청 수신, YOLO 추론 시작...")

        # 1. 신뢰도(conf)를 0.6로 강하게 올립니다.
        results = self.model(self.current_ir_img, conf=0.6, verbose=False)
        
        found_positions = []
        h_img, w_img = self.current_depth_img.shape # 전체 이미지 크기
        
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            detected_name = self.model.names[cls_id]
            
            # =========================================================
            # [핵심 수정] 요청이 'brass_roi'일 때는 'brass'를 찾도록 매핑
            # =========================================================
            is_matching = False
            if target_name == "brass_roi" and detected_name == "brass":
                is_matching = True
            elif detected_name == target_name:
                is_matching = True
                
            if is_matching:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # --- [필터링 1] 바운딩 박스가 너무 작거나 크면 노이즈로 간주하고 무시 ---
                box_width, box_height = x2 - x1, y2 - y1
                if box_width < 10 or box_height < 10: 
                    continue 
                if box_width > 400 or box_height > 400:
                    continue 
                
                u, v = (x1 + x2) // 2, (y1 + y2) // 2
                
                # =========================================================
                # [핵심 수정] 1차 탐색(brass_roi)일 때만 ROI 영역 필터링 적용
                # ROI 좌표: x(113~640), y(9~480)
                # =========================================================
                if target_name == "brass_roi":
                    if not (113 <= u <= 640 and 9 <= v <= 480):
                        self.get_logger().info(f"🚫 ROI 영역 밖 탄피 무시됨 (픽셀 좌표: x={u}, y={v})")
                        continue
                
                # --- [필터링 2] 중심점(u, v)이 이미지 맨 가장자리면 무시 ---
                MARGIN = 15 
                if u < MARGIN or u > (w_img - MARGIN) or v < MARGIN or v > (h_img - MARGIN):
                    self.get_logger().warn(f"[{u},{v}] 가장자리 노이즈 감지. 건너뜁니다.")
                    continue
                
                # --- 뎁스 추출 ---
                v_min, v_max = max(0, v-3), min(h_img, v+4) 
                u_min, u_max = max(0, u-3), min(w_img, u+4)
                
                depth_window = self.current_depth_img[v_min:v_max, u_min:u_max]
                valid_depths = depth_window[depth_window > 0] 
                
                if len(valid_depths) == 0:
                    self.get_logger().warn(f"[{u},{v}] 주변 뎁스 값이 모두 깨져서 건너뜁니다.")
                    continue
                
                depth_mm = float(np.median(valid_depths)) 

                # 뎁스 값이 비정상적으로 크거나 작으면 무시 (예: 20cm 미만, 2m 이상)
                if depth_mm < 200 or depth_mm > 2000:
                    continue

                fx, fy = self.intrinsics['fx'], self.intrinsics['fy']
                cx, cy = self.intrinsics['cx'], self.intrinsics['cy']

                target_x = (u - cx) * depth_mm / fx
                target_y = (v - cy) * depth_mm / fy
                target_z = depth_mm
                
                found_positions.extend([float(target_x), float(target_y), float(target_z)])
                
        response.depth_position = found_positions
        
        count = len(found_positions) // 3
        if count > 0:
            self.get_logger().info(f"✅ {target_name} 총 {count}개 발견 완료!")
        else:
            self.get_logger().info(f"❌ {target_name}을(를) 찾을 수 없거나 유효한 뎁스가 없습니다.")
            
        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()