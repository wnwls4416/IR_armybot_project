import sys
import os
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from flask import Flask, render_template, jsonify

app = Flask(__name__)

# 전역 변수 (상태 저장용)
current_shot_count = 0     
analysis_result = None     
is_jammed = False          # 기능고장 상태 저장
is_shocked = False         # [추가] 대기 상태 저장

# ---------------------------------------------------------
# ROS 2 노드
# ---------------------------------------------------------
class ShooterWebNode(Node):
    def __init__(self):
        super().__init__('shooter_web_node')
        
        self.sub_shoot = self.create_subscription(Int32, '/signal_shoot', self.listener_callback, 10)
        self.sub_check = self.create_subscription(String, '/check_data', self.analysis_callback, 10)
        self.sub_restart = self.create_subscription(Int32, '/signal_restart', self.restart_callback, 10)
        
        # 기능고장 관련 구독
        self.sub_jammed = self.create_subscription(Int32, '/jammed', self.jammed_callback, 10)
        self.sub_jammed_clear = self.create_subscription(Int32, '/jammed_clear', self.jammed_clear_callback, 10)

        # [추가] 대기(Shocked) 관련 구독
        self.sub_shocked_5 = self.create_subscription(Int32, '/shocked_five', self.shocked_callback, 10)
        self.sub_shocked_3 = self.create_subscription(Int32, '/shocked_three', self.shocked_callback, 10)
        self.sub_shocked_solved = self.create_subscription(Int32, '/shocked_solved', self.shocked_clear_callback, 10)
        
        self.get_logger().info('🔫 SHOOTER HUD SYSTEM ONLINE.')

    def listener_callback(self, msg):
        global current_shot_count
        current_shot_count = msg.data
        self.get_logger().info(f'💥 SHOT FIRED! Count: {current_shot_count}/7')

    def analysis_callback(self, msg):
        global analysis_result
        analysis_result = msg.data
        self.get_logger().info(f'🎯 ANALYSIS DATA RECEIVED: {analysis_result}')

    def restart_callback(self, msg):
        global current_shot_count, analysis_result
        if msg.data == 1:
            current_shot_count = 0
            analysis_result = None
            self.get_logger().info('🔄 SYSTEM RESTART RECEIVED: Ammo Reset.')

    def jammed_callback(self, msg):
        global is_jammed
        if msg.data == 1:
            is_jammed = True
            self.get_logger().info('⚠️ 기능고장 (JAMMED) 발생!')

    def jammed_clear_callback(self, msg):
        global is_jammed
        if msg.data == 1:
            is_jammed = False
            self.get_logger().info('✅ 기능고장 조치 완료 (JAMMED CLEAR)!')

    # [추가] 대기 발생 콜백
    def shocked_callback(self, msg):
        global is_shocked
        if msg.data == 1:
            is_shocked = True
            self.get_logger().info('⏳ 로봇 대기 (SHOCKED) 상태 수신!')

    # [추가] 대기 해제 콜백
    def shocked_clear_callback(self, msg):
        global is_shocked
        if msg.data == 1:
            is_shocked = False
            self.get_logger().info('▶️ 로봇 대기 해제 (SHOCKED SOLVED)!')

# ROS 노드 객체
ros_node = None

# ---------------------------------------------------------
# Flask 웹 서버 라우팅
# ---------------------------------------------------------
@app.route('/')
def index():
    return render_template('shooter.html')

@app.route('/status')
def get_status():
    global current_shot_count, analysis_result, is_jammed, is_shocked
    return jsonify({
        'shots': current_shot_count,
        'analysis': analysis_result,
        'is_jammed': is_jammed,
        'is_shocked': is_shocked  # [추가] 상태 전송
    })

@app.route('/reset', methods=['POST'])
def reset_counter():
    global current_shot_count, analysis_result
    current_shot_count = 0
    analysis_result = None
    return jsonify({'result': 'ok'})

# ---------------------------------------------------------
# 메인 실행
# ---------------------------------------------------------
def run_ros():
    rclpy.spin(ros_node)

if __name__ == '__main__':
    rclpy.init()
    ros_node = ShooterWebNode()
    threading.Thread(target=run_ros, daemon=True).start()
    app.run(host='0.0.0.0', port=5001, debug=False)