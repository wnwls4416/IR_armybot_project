import sys
import os
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from flask import Flask, render_template, jsonify

# API 키 설정
MY_API_KEY = "GPT_API"
os.environ["OPENAI_API_KEY"] = MY_API_KEY

from langchain_openai import ChatOpenAI
from langchain_core.prompts import PromptTemplate
from stt import STT

app = Flask(__name__)

# [상태 저장용 전역 변수]
is_jammed = False
is_shocked = False  # [추가] 대기(Shocked) 상태

# ---------------------------------------------------------
# 1. ROS 2 노드
# ---------------------------------------------------------
class CommanderWebNode(Node):
    def __init__(self):
        super().__init__('commander_web_node')
        qos_profile = 1
        
        # 발행(Publish) 토픽들
        self.pub_give = self.create_publisher(Int32, '/magazine_give', qos_profile)
        self.pub_start = self.create_publisher(Int32, '/signal_start', qos_profile)
        self.pub_take = self.create_publisher(Int32, '/magazine_take', qos_profile)
        self.pub_brass = self.create_publisher(Int32, '/check_brass', qos_profile)
        self.pub_restart = self.create_publisher(Int32, '/signal_restart', qos_profile)

        # 구독(Subscribe) 토픽들: 기능고장
        self.sub_jammed = self.create_subscription(Int32, '/jammed', self.jammed_callback, 10)
        self.sub_jammed_clear = self.create_subscription(Int32, '/jammed_clear', self.jammed_clear_callback, 10)

        # [추가] 구독(Subscribe) 토픽들: 대기(Shocked)
        self.sub_shocked_5 = self.create_subscription(Int32, '/shocked_five', self.shocked_callback, 10)
        self.sub_shocked_3 = self.create_subscription(Int32, '/shocked_three', self.shocked_callback, 10)
        self.sub_shocked_solved = self.create_subscription(Int32, '/shocked_solved', self.shocked_clear_callback, 10)

    def send_command(self, keyword):
        msg = Int32()
        msg.data = 1
        if keyword == "magazine_give":
            self.pub_give.publish(msg)
            return "✅ [전송] 탄알집 인계"
        elif keyword == "signal_start":
            self.pub_start.publish(msg)
            return "✅ [전송] 사격 개시"
        elif keyword == "magazine_take":
            self.pub_take.publish(msg)
            return "✅ [전송] 탄알집 회수"
        elif keyword == "check_brass":
            self.pub_brass.publish(msg)
            return "✅ [전송] 탄피 확인"
        return "❌ 알 수 없는 명령"

    def send_restart_signal(self):
        msg = Int32()
        msg.data = 1
        self.pub_restart.publish(msg)
        return "🔄 [전송] 시스템 리셋 (Shooter UI 초기화)"

    # 기능고장 콜백
    def jammed_callback(self, msg):
        global is_jammed
        if msg.data == 1:
            is_jammed = True
            self.get_logger().info('⚠️ 기능고장 (JAMMED) 수신!')

    def jammed_clear_callback(self, msg):
        global is_jammed
        if msg.data == 1:
            is_jammed = False
            self.get_logger().info('✅ 기능고장 조치 완료 (JAMMED CLEAR) 수신!')

    # [추가] 대기 콜백
    def shocked_callback(self, msg):
        global is_shocked
        if msg.data == 1:
            is_shocked = True
            self.get_logger().info('⏳ 대기 상태 (SHOCKED) 수신!')

    def shocked_clear_callback(self, msg):
        global is_shocked
        if msg.data == 1:
            is_shocked = False
            self.get_logger().info('▶️ 대기 해제 (SHOCKED SOLVED) 수신!')

ros_node = None

# ---------------------------------------------------------
# 2. AI 처리 로직
# ---------------------------------------------------------
def process_voice_command():
    try:
        stt = STT(openai_api_key=MY_API_KEY)
        llm = ChatOpenAI(model="gpt-4o", temperature=0, openai_api_key=MY_API_KEY)

        prompt_content = """
            당신은 사격장 로봇 통제관입니다. 
            사용자의 불분명한 발음이나 오타를 문맥에 맞게 추론하여, 아래 4가지 핵심 명령어 중 하나로 변환하세요.

            <목표>
            사용자의 입력이 다음 중 어떤 의도인지 파악하여 **오직 영어 키워드 하나만** 출력하세요.

            <명령어 리스트>
            1. magazine_give (의미: 탄알집 인계, 탄창 줘, 탄알집 전달, 탄알집 인게)
            2. signal_start (의미: 사격 개시, 사격 시작, 쏴, 발사)
            3. magazine_take (의미: 탄알집 회수, 탄창 가져가, 탄알집 제거)
            4. check_brass (의미: 탄피 확인, 탄피 체크, 탄피 몇 개야)

            <특수 규칙>
            - 발음이 비슷하거나 오타가 있어도 최대한 위 4개 중 하나로 매칭하세요.
            - 도저히 알 수 없는 말이면 "unknown" 출력.

            <사용자 입력>
            "{user_input}"
        """
        
        prompt = PromptTemplate(input_variables=["user_input"], template=prompt_content)
        chain = prompt | llm

        text = stt.speech2text()
        print(f"🗣️ 인식된 텍스트: {text}")

        response = chain.invoke({"user_input": text})
        keyword = response.content.strip()
        print(f"🤖 분석 결과: {keyword}")

        return text, keyword

    except Exception as e:
        print(f"⚠️ 에러: {e}")
        return str(e), "error"

# ---------------------------------------------------------
# 3. Flask 라우팅
# ---------------------------------------------------------
@app.route('/')
def index():
    return render_template('commander.html')

@app.route('/execute_command', methods=['POST'])
def execute_command():
    text, keyword = process_voice_command()
    result_msg = ""
    if keyword == "error":
        result_msg = "⚠️ 시스템 오류"
    elif keyword == "unknown":
        result_msg = "❌ 명령 불명확"
    elif ros_node:
        result_msg = ros_node.send_command(keyword)
    else:
        result_msg = "⚠️ ROS 노드 미작동"

    return jsonify({'text': text, 'keyword': keyword, 'result': result_msg})

@app.route('/send_restart', methods=['POST'])
def send_restart():
    result_msg = ""
    if ros_node:
        result_msg = ros_node.send_restart_signal()
    else:
        result_msg = "⚠️ ROS 노드 미작동"
    return jsonify({'result': result_msg})

@app.route('/status')
def get_status():
    global is_jammed, is_shocked
    # [추가] is_shocked 데이터 전송
    return jsonify({'is_jammed': is_jammed, 'is_shocked': is_shocked})

def run_ros():
    rclpy.spin(ros_node)

if __name__ == '__main__':
    rclpy.init()
    ros_node = CommanderWebNode()
    threading.Thread(target=run_ros, daemon=True).start()
    app.run(host='0.0.0.0', port=5000, debug=False)