#!/usr/bin/env python3
import serial
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class IntegratedGunshotPub(Node):
    def __init__(self):
        super().__init__('integrated_gunshot_pub')

        # 아두이노 시리얼 설정
        self.port = self.declare_parameter('port', '/dev/ttyACM0').value
        self.baud = self.declare_parameter('baud', 115200).value
        self.on_value = self.declare_parameter('on_value', 0).value
        
        # 발사 카운터 설정 (1~7)
        self.min_val = int(self.declare_parameter('min_val', 1).value)
        self.max_val = int(self.declare_parameter('max_val', 7).value)
        self.counter = int(self.declare_parameter('start_val', 1).value)

        # 오디오 재생 설정
        self.wav_path = self.declare_parameter('wav', '/home/rokey/cobot2_ws/src/sounds/gunshot_std.wav').value
        self.cooldown_sec = float(self.declare_parameter('cooldown', 0.2).value)
        self.last_play_time = 0.0

        # 전역 토픽 퍼블리셔
        self.pub = self.create_publisher(Int32, '/signal_shoot', 10)

        # 시리얼 포트 열기
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        self.get_logger().info(
            f"통합 노드 시작됨: 포트={self.port}, 사운드={self.wav_path}, 카운터={self.min_val}~{self.max_val}"
        )

        self.last = None
        # 0.01초마다 스위치 상태를 확인하는 타이머
        self.timer = self.create_timer(0.01, self.poll)

    def poll(self):
        # 시스템 멈춤 방지를 위한 데이터 수신 대기 확인
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line not in ('0', '1'):
                return
            cur = int(line)

            if self.last is None:
                self.last = cur
                return

            # 스위치가 눌린 순간(Edge) 감지
            if self.last != self.on_value and cur == self.on_value:
                now = self.get_clock().now().nanoseconds / 1e9
                
                # 쿨다운 시간이 지났을 때만 실행 (중복 발사 방지)
                if (now - self.last_play_time) >= self.cooldown_sec:
                    self.last_play_time = now

                    # 1. 총소리 재생
                    try:
                        subprocess.Popen(['paplay', self.wav_path])
                    except Exception as e:
                        self.get_logger().error(f"사운드 재생 실패: {e}")

                    # 2. 토픽 발행
                    msg = Int32()
                    msg.data = self.counter
                    self.pub.publish(msg)
                    self.get_logger().info(f"빵! 소리 재생 및 토픽 발행됨 -> /signal_shoot: {msg.data}")

                    # 3. 카운터 증가 (7 넘으면 1로 초기화)
                    self.counter += 1
                    if self.counter > self.max_val:
                        self.counter = self.min_val

            self.last = cur

def main():
    rclpy.init()
    node = IntegratedGunshotPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()