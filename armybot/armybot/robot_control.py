import os
import time
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from od_msg.srv import SrvDepthPosition
import numpy as np
from std_msgs.msg import Int32, String
import threading
from scipy.spatial.transform import Rotation

# 전역 변수 초기화
movej = None
movel = None
mwait = None
get_external_torque = None 
get_current_posx = None
set_tcp = None
set_tool = None

get_robot_state = None      
drl_script_stop = None      
set_robot_control = None    
DR_QSTOP_STO = None


def force_inject_dsr(node):
    global movej, movel, mwait, get_external_torque, get_current_posx, get_robot_state, drl_script_stop, DR_QSTOP_STO
    
    # 이전에 임포트된 DSR 모듈 캐시를 제거하여 재로드 보장
    for key in ['DR_init', 'DSR_ROBOT2']:
        if key in sys.modules:
            del sys.modules[key]

    # 'armybot' 패키지 경로 및 현재 디렉터리를 sys.path에서 임시 제거
    original_path = sys.path.copy()
    sys.path = [p for p in sys.path if 'armybot' not in p and p not in ('', '.')]

    try:
        import DR_init
        DR_init.__dsr__id = "dsr01"
        DR_init.__dsr__model = "m0609"
        DR_init.__dsr__node = node
        
        import DSR_ROBOT2
        movej = DSR_ROBOT2.movej
        movel = DSR_ROBOT2.movel
        mwait = DSR_ROBOT2.mwait
        get_external_torque = DSR_ROBOT2.get_external_torque
        get_current_posx = DSR_ROBOT2.get_current_posx 
        set_tcp = DSR_ROBOT2.set_tcp
        set_tool = DSR_ROBOT2.set_tool
        get_robot_state = DSR_ROBOT2.get_robot_state
        drl_script_stop = DSR_ROBOT2.drl_script_stop

        DR_QSTOP_STO = DSR_ROBOT2.DR_QSTOP_STO
        
        node.get_logger().info(f"✅ DSR 라이브러리 로드 성공 (경로: {DR_init.__file__})")
        return True
    except Exception as e:
        node.get_logger().error(f"❌ DSR 로드 실패: {e}")
        return False
    finally:
        sys.path = original_path

class RobotMissionNode(Node):
    def __init__(self):
        self.ROBOT_ID = 'dsr01'
        self.ROBOT_MODEL = 'm0609'
        super().__init__('robot_mission_executor', namespace=self.ROBOT_ID)

        # [에러 복구 관련 변수 초기화]
        self.is_error_recovery_mode = False     
        self.last_robot_state = 1               
        self.CONTROL_RESET_SAFE_STOP = 2        
        self.CONTROL_RESET_SAFE_OFF = 3         
        self.CONTROL_RESET_RECOVERY = 7         

        self.get_logger().info("🚀 로봇 미션 노드 초기화 완료 및 안전 감시 시작")        
        self.current_mission = None
        self.VEL, self.ACC = 60, 60
        self.USER_HANDOVER_POS = [8.34, 7.16, 51.42, -0.12, 91.65, -0.73]       
        self.BUCKET_POS = [39.69, -6.54, 104.96, 0.02, 81.61, 39.69]            
        self.READY_POSE = [4.46, 7.32, 50.38, -0.04, 122.30, -0.97]             
        self.THROW_AWAY = [61.24, 27.90, 80.68, 3.30, 72.44, -0.97]             
        self.SHOOT_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]                        
        self.gripper = None

        self.is_tap_mode = False       
        self.tap_baseline = 0.0        
        self.tap_step = 0              
        self.DIFF_THRESHOLD = 15.0     
        self.POS_TAP_1 = [0, 0, 45, 0, 130, 0]          
        self.POS_TAP_2 = [-70, 90, 0, -270, -15, 0]     

        self.create_subscription(Int32, '/magazine_give', self.cb_magazine_give, 10)        
        self.create_subscription(Int32, '/magazine_take', self.cb_magazine_take, 10)        
        self.create_subscription(Int32, '/signal_start', self.cb_signal_start, 10)          
        self.create_subscription(Int32, '/check_brass', self.cb_check_brass, 10)            
        self.create_subscription(Int32, '/signal_restart', self.cb_signal_restart, 10)      
        self.create_subscription(Int32, '/signal_shoot', self.cb_signal_shoot, 10)          
    
        self.pub_trigger_ai = self.create_publisher(Int32, '/trigger_ai_count', 10)         
        self.pub_alert = self.create_publisher(String, '/robot_alert', 10)                  
        self.pos_client = self.create_client(SrvDepthPosition, "/get_3d_position")          

        self.pub_jammed = self.create_publisher(Int32, '/jammed', 10)                       
        self.pub_jammed_clear = self.create_publisher(Int32, '/jammed_clear', 10)           
        
        self.pub_shocked_five = self.create_publisher(Int32, '/shocked_five', 10)           
        self.pub_shocked_three = self.create_publisher(Int32, '/shocked_three', 10)         
        self.pub_shocked_solved = self.create_publisher(Int32, '/shocked_solved', 10)       

        self.get_logger().info("🚀 로봇 미션 노드 초기화 완료")

    def init_gripper(self):
        try:
            from armybot.onrobot import RG
            self.gripper = RG("rg2", "192.168.1.1", "502")
            self.get_logger().info("✅ RG 그리퍼 연결 완료")
        except ImportError:
            self.get_logger().warn("⚠️ RG 그리퍼 라이브러리 로드 실패")

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R           
        T[:3, 3] = [x, y, z]    
        return T

    def transform_to_base(self, camera_coords, robot_pos):
        npy_path = '/home/rokey/cobot2_ws/src/armybot/resource/T_gripper2camera.npy'
        try:
            gripper2cam = np.load(npy_path)                             
        except Exception as e:
            self.get_logger().error(f"캘리브레이션 파일 로드 실패: {e}")
            return [0.0, 0.0, 0.0]
        
        coord = np.append(np.array(camera_coords), 1) 
        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)      
        base2cam = base2gripper @ gripper2cam                               
        target_base_coord = np.dot(base2cam, coord)                         
        return target_base_coord[:3]

    def cb_magazine_give(self, msg): self.current_mission = 'GIVE'          
    def cb_magazine_take(self, msg): self.current_mission = 'TAKE'          
    def cb_signal_start(self, msg): self.current_mission = 'START_SIGNAL'   
    def cb_check_brass(self, msg): self.current_mission = 'CHECK_BRASS'     
    def cb_signal_restart(self, msg): self.current_mission = 'RESTART'      
    
    def cb_signal_shoot(self, msg):
        if msg.data == 7:
            self.current_mission = 'SHOOT'
        else: 
            self.get_logger().warn(f"⚠️ 사격 종료 신호 아님 (무시됨): {msg.data}")

    def get_current_impact(self):
        if not get_external_torque: return 0.0
        try:
            torques = get_external_torque()
            return sum([abs(t) for t in torques]) if torques else 0.0       
        except: return 0.0

    def process_mission(self):
        if not movej: return

        if self.is_error_recovery_mode:
            return
        
        if self.current_mission:
            mission = self.current_mission
            self.current_mission = None 
            if mission != 'START_SIGNAL': self.is_tap_mode = False

            try:
                if mission == 'SHOOT':
                    self.get_logger().info("📸 사격 좋료 위치 이동 전, AI 비전 노드에 표적지 촬영 지시를 보냅니다!")
                    trigger_msg = Int32()
                    trigger_msg.data = 1
                    self.pub_trigger_ai.publish(trigger_msg) 
                    
                    self.get_logger().info("▶ 카메라가 흔들림 없이 찍을 수 있도록 2초 대기합니다...")
                    time.sleep(2.0) 
                    
                    self.get_logger().info("▶ 사격 종료 위치로 이동 시작")
                    movej(self.SHOOT_POSE, vel=120, acc=80)
                    self.wait_robot()
                    
                    self.get_logger().info("▶ 도착 완료! 2초 대기 중...")
                    self.safe_sleep(2.0)

                    self.go_ready_pose()

                elif mission == 'GIVE':
                    if self.gripper: self.gripper.open_gripper()
                    time.sleep(0.5)
                    movej(self.BUCKET_POS, vel=self.VEL, acc=self.ACC)
                    if self.gripper: self.gripper.close_gripper()
                    time.sleep(0.5)
                    if get_current_posx and movel:
                        posx, _ = get_current_posx()
                        posx[2] += 200.0             
                        movel(posx, vel=self.VEL, acc=self.ACC) 
                    movej(self.USER_HANDOVER_POS, vel=self.VEL, acc=self.ACC)
                    time.sleep(1.0)
                    if self.gripper: self.gripper.open_gripper()
                    time.sleep(0.5)
                    self.go_ready_pose()

                elif mission == 'TAKE':
                    self.get_logger().info("🔍 탄알집 확인 위치(READY_POSE)로 이동합니다.")
                    movej(self.READY_POSE, vel=self.VEL, acc=self.ACC)
                    self.wait_robot()
                    time.sleep(3.0) 

                    self.get_logger().info("👀 탄알집 탐색을 시작합니다.")
                    target_pos = self.call_vision_service("magazine") 

                    if target_pos:
                        self.get_logger().info("✅ 탄알집 발견! 수거(Pick & Place)를 시작합니다.")
                        approach_pos = list(target_pos)
                        approach_pos[2] += 100.0 
                        pick_pos = list(target_pos)
                        pick_pos[2] += 60.0

                        movel(approach_pos, vel=self.VEL, acc=self.ACC)
                        movel(pick_pos, vel=30, acc=30)
                        if self.gripper: self.gripper.close_gripper()
                        self.wait_robot()
                        time.sleep(1.0) 
                        movej(self.READY_POSE, vel=self.VEL, acc=self.ACC)
                        self.wait_robot()

                        self.get_logger().info("⬆️ READY_POSE 도착 후 Z축으로 50mm 상승합니다.")
                        if get_current_posx and movel:
                            pos_x, _ = get_current_posx()
                            pos_x[2] += 50.0
                            movel(pos_x, vel=self.VEL, acc=self.ACC)
                            self.wait_robot()

                        self.get_logger().info("🗑️ 탄알집을 보관함으로 이동합니다.")
                        movej(self.THROW_AWAY, vel=self.VEL, acc=self.ACC)
                        self.wait_robot()
                        if self.gripper: self.gripper.open_gripper()
                        time.sleep(1.0)
                    else:
                        self.get_logger().error("❌ 탄알집을 찾지 못했습니다.")

                    self.go_ready_pose()

                elif mission == 'START_SIGNAL':
                    movej(self.POS_TAP_2, vel=120, acc=80)
                    time.sleep(2.0)
                    self.is_tap_mode, self.tap_step = True, 0
                    self.tap_baseline = self.get_current_impact()
                    self.get_logger().info(f"✅ 기능고장 탭 모드 활성화 (기준: {self.tap_baseline:.2f} Nm)")

                elif mission == 'CHECK_BRASS':
                    SCAN_POSE = [-0.56, 27.67, 55.26, -0.05, 97.07, 0.46]
                    FIND_POSE = [-0.10, -15.94, 100.06, 0.01, 95.88, -0.12]
                    target_count = 7

                    self.get_logger().info("🔍 1차 탄피 확인 위치로 이동합니다.")
                    movej(SCAN_POSE, vel=self.VEL, acc=self.ACC)
                    self.wait_robot()
                    time.sleep(3) 

                    # =========================================================
                    # [핵심 수정] 1차 스캔: ROI 필터링을 거친 "brass_roi" 호출
                    # =========================================================
                    brass_list = self.call_vision_service_for_multiple_targets("brass_roi")
                    current_count = len(brass_list) if brass_list else 0

                    if current_count >= target_count:
                        self.get_logger().info(f"✅ 목표 수량 충족! (현재 {current_count}개 발견)")
                        self.go_ready_pose()
                    else: 
                        self.get_logger().warn(f"⚠️ 탄피 수량 부족 ({current_count}/{target_count}). 추가 탐색을 시작합니다.")
                        movej(FIND_POSE, vel=self.VEL, acc=self.ACC)
                        self.wait_robot()
                        time.sleep(1.0) 

                        # =========================================================
                        # [핵심 수정] 2차 스캔: 전체 화면을 탐색하는 일반 "brass" 호출
                        # =========================================================
                        new_brass_list = self.call_vision_service_for_multiple_targets("brass")
                        new_count = len(new_brass_list) if new_brass_list else 0

                        if new_count > 0:
                            self.get_logger().info(f"👀 추가 탐색에서 {new_count}개의 탄피를 발견했습니다! 수거(Pick & Place)를 시작합니다.")
                            for i, brass_pos in enumerate(new_brass_list):
                                self.get_logger().info(f"[{i+1}/{new_count}] 번째 탄피 수거 중...")
                                approach_pos = list(brass_pos)
                                approach_pos[2] += 40.0 
                                self.gripper.open_gripper()
                                movel(approach_pos, vel=self.VEL, acc=self.ACC)
                                movel(brass_pos, vel=30, acc=30)
                                if self.gripper: self.gripper.close_gripper()
                                self.wait_robot()
                                time.sleep(1.0) 
                                movej(approach_pos, vel=self.VEL, acc=self.ACC)
                                movej(self.READY_POSE, vel=self.VEL, acc = self.ACC)
                                movej(SCAN_POSE, vel=self.VEL, acc=self.ACC)
                                self.wait_robot()
                                if self.gripper: self.gripper.open_gripper()
                                time.sleep(1.0)

                                if i < new_count - 1:
                                    movej(FIND_POSE, vel=self.VEL, acc=self.ACC)
                                    self.wait_robot()
                        else:
                            self.get_logger().error("❌ 추가 탐색 위치(FIND_POSE)에서도 남은 탄피를 찾지 못했습니다.")
                        self.go_ready_pose()

                    self.get_logger().info("📦 탄피 수거 완료! 통을 잡으러 이동합니다.")
                    GRAB_BASKET_POSE = [12.88, 12.21, 72.63, -0.03, 95.15, 7.45]
                    movej(GRAB_BASKET_POSE, vel = self.VEL, acc = self.ACC)
                    self.wait_robot()

                    self.get_logger().info("⬇️ Z축으로 30mm 내려갑니다.")
                    if get_current_posx and movel:
                        pos_x, _ = get_current_posx()
                        pos_x[2] -= 50.0  
                        movel(pos_x, vel=30, acc=30) 
                        self.wait_robot()
                    
                    if self.gripper: self.gripper.close_gripper()
                    time.sleep(1.0) 
                    self.get_logger().info("🏠 통을 잡고 대기(READY) 위치로 이동합니다.")
                    movej(self.READY_POSE, vel=self.VEL, acc=self.ACC)
                    self.wait_robot()

                    HAND_OVER = [42.79, 29.20, 35.24, -0.07, 115.37, 36.91]
                    movej(HAND_OVER, vel = self.VEL, acc = self.ACC)
                    self.wait_robot()
                    self.gripper.open_gripper()
                    movej(self.READY_POSE, vel = self.VEL, acc = self.ACC)
                    self.gripper.close_gripper()

                elif mission == 'RESTART':
                    self.go_ready_pose()
            except Exception as e:
                self.get_logger().error(f"오류: {e}")

        elif self.is_tap_mode:
            current_val = self.get_current_impact()
            if abs(current_val - self.tap_baseline) >= self.DIFF_THRESHOLD:
                msg = Int32()
                msg.data = 1
                if self.tap_step == 0:
                    self.pub_jammed.publish(msg)
                    movej(self.POS_TAP_1, vel=120, acc=80)
                    self.tap_step = 1
                else:
                    self.pub_jammed_clear.publish(msg)
                    movej(self.POS_TAP_2, vel=120, acc=80)
                    self.tap_step = 0
                time.sleep(2.0)
                self.tap_baseline = self.get_current_impact()

    def go_ready_pose(self):
        movej(self.READY_POSE, vel=self.VEL, acc=self.ACC)
        if self.gripper: self.gripper.open_gripper()
        mwait()

    def call_set_robot_control(self, cmd):
        import subprocess
        import threading
        
        self.get_logger().info(f"🔄 에러 초기화 명령(코드: {cmd}) 전송 시도 중...")
        
        def run_reset_cmd():
            cmd_str = f"ros2 service call /{self.ROBOT_ID}/system/set_robot_control dsr_msgs2/srv/SetRobotControl \"{{robot_control: {cmd}}}\""
            try:
                result = subprocess.run(cmd_str, shell=True, executable='/bin/bash', capture_output=True, text=True)
                
                if result.returncode == 0:
                    self.get_logger().info(f"📩 서비스 응답 수신: {result.stdout.strip()}")
                else:
                    self.get_logger().error(f"❌ 터미널 명령 실패: {result.stderr.strip()}")
            except Exception as e:
                self.get_logger().error(f"명령 실행 중 파이썬 오류: {e}")
                
        threading.Thread(target=run_reset_cmd, daemon=True).start()

    def call_vision_service(self, target_name):
        if not self.pos_client.wait_for_service(timeout_sec=1.0): 
            self.get_logger().warn("⚠️ 비전 서비스가 응답하지 않습니다.")
            return None 
            
        req = SrvDepthPosition.Request()
        req.target = target_name
        future = self.pos_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        
        if future.done():
            res = future.result()
            dp = res.depth_position
            
            if len(dp) >= 3:
                cam_x, cam_y, cam_z = dp[0], dp[1], dp[2]
                if get_current_posx is not None:
                    current_posx, _ = get_current_posx()
                    base_xyz = self.transform_to_base([cam_x, cam_y, cam_z], current_posx)
                    target_z = base_xyz[2] - 5.0 
                    target_z = max(target_z, 2.0)
                    target_pos = [
                        base_xyz[0], base_xyz[1], target_z, 
                        current_posx[3], current_posx[4], current_posx[5]
                    ]
                    self.get_logger().info(f"👉 단일 타겟 변환 완료: X:{target_pos[0]:.1f}, Y:{target_pos[1]:.1f}, Z:{target_pos[2]:.1f}")
                    return target_pos
        return None
    
    def call_vision_service_for_multiple_targets(self, target_name):
        if not self.pos_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("⚠️ 비전 서비스가 응답하지 않습니다.")
            return []
        req = SrvDepthPosition.Request()
        req.target = target_name
        future = self.pos_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.done():
            res = future.result()
            target_list = []
            dp = res.depth_position
            num_targets = len(dp) // 3

            if num_targets > 0 :
                self.get_logger().info(f"{target_name} {num_targets}개 좌표 수신 완료.")
                if get_current_posx is None:
                    return []
                current_posx, _ = get_current_posx()

                for i in range(0, len(dp), 3):
                    cam_x, cam_y, cam_z = dp[i], dp[i+1], dp[i+2]
                    base_xyz = self.transform_to_base([cam_x, cam_y, cam_z], current_posx)
                    target_z = base_xyz[2] - 5.0  
                    target_z = max(target_z, 2.0) 
                    target_list.append([
                        base_xyz[0], base_xyz[1], target_z, 
                        current_posx[3], current_posx[4], current_posx[5]  
                    ])
                return target_list
            else :
                self.get_logger().info(f"{target_name}이(가) 검출되지 않았습니다.")
                return []
        return []
    
    def check_and_recover_robot(self):
        if not get_robot_state: return
        try:
            state = get_robot_state()
            if state is not None:
                self.last_robot_state = state

            if state in [3, 5, 6]:
                if not self.is_error_recovery_mode:
                    self.get_logger().error(f"🚨 로봇 에러 감지! (상태 코드: {state}) - 복구 시퀀스 가동")
                    self.is_error_recovery_mode = True
                    self.recovery_start_time = time.time()
                    self.current_mission = None 
                    
                    topic_msg = Int32()
                    topic_msg.data = 1
                    if state == 5:
                        self.pub_shocked_five.publish(topic_msg)
                        self.get_logger().info("📤 가벼운 충돌! /shocked_five 토픽 발송 완료")
                    elif state == 3:
                        self.pub_shocked_three.publish(topic_msg)
                        self.get_logger().info("📤 심각한 정지! /shocked_three 토픽 발송 완료")

                    if drl_script_stop and DR_QSTOP_STO is not None:
                        drl_script_stop(DR_QSTOP_STO)
                    time.sleep(1.0) 
                    
                    if state == 3:
                        cmd = self.CONTROL_RESET_SAFE_OFF
                    elif state == 5:
                        cmd = self.CONTROL_RESET_SAFE_STOP
                    else:
                        cmd = self.CONTROL_RESET_RECOVERY
                        
                    self.call_set_robot_control(cmd)
                else:
                    if time.time() - self.recovery_start_time > 3.0:
                        self.get_logger().warn(f"⏳ 상태 {state}에서 로봇이 멈춰있습니다. 리셋 명령을 재전송합니다...")
                        
                        if state == 3:
                            cmd = self.CONTROL_RESET_SAFE_OFF
                        elif state == 5:
                            cmd = self.CONTROL_RESET_SAFE_STOP
                        else:
                            cmd = self.CONTROL_RESET_RECOVERY
                            
                        self.call_set_robot_control(cmd)
                        self.recovery_start_time = time.time()

            elif self.is_error_recovery_mode:
                if state == 1:
                    self.get_logger().info("✅ 로봇 정상(Standby) 상태로 복구 완료! 대기 위치로 이동합니다.")
                    self.is_error_recovery_mode = False
                    
                    alert_msg = String()
                    alert_msg.data = "✅ 로봇 복구 완료. 정상 작동합니다."
                    self.pub_alert.publish(alert_msg)
                    
                    solved_msg = Int32()
                    solved_msg.data = 1
                    self.pub_shocked_solved.publish(solved_msg)
                    self.get_logger().info("📤 복구 성공! /shocked_solved 토픽 발송 완료")
                    
                    try:
                        self.go_ready_pose()        
                    except Exception as e:
                        self.get_logger().warn(f"복구 후 READY 이동 실패: {e}")
                else:
                    if not hasattr(self, 'last_log_state') or self.last_log_state != state:
                        self.get_logger().info(f"🔄 로봇이 상태 {state}(으)로 전환되었습니다. 완전한 대기 상태(1)를 기다립니다...")
                        self.last_log_state = state

        except Exception as e:
            self.get_logger().warn(f"상태 확인 중 오류: {e}")

    def check_mission_error(self):
        if not get_robot_state: return
        state = get_robot_state()
        if state in [3, 5, 6]:
            raise RuntimeError(f"ROBOT_ERROR_{state}") 

    def wait_robot(self):
        if mwait: mwait()
        self.check_mission_error()

    def safe_sleep(self, duration):
        start = time.time()
        while time.time() - start < duration:
            self.check_mission_error()
            time.sleep(0.1)
             
def main(args=None):
    rclpy.init(args=args)
    node = RobotMissionNode()
    if force_inject_dsr(node):
        node.init_gripper()
        try:
            if set_tcp and set_tool :
                set_tcp('GripperDA_v1')
                set_tool('Tool Weight')
                        
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.01)
                node.check_and_recover_robot() 
                node.process_mission()         
        except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()