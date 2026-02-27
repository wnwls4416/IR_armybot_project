import speech_recognition as sr
import subprocess
from gtts import gTTS
import os
import time
import socket

def get_my_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip

def speak(text):
    print(f"자비스: {text}")
    tts = gTTS(text=text, lang='ko')
    tts.save("voice.mp3")
    os.system("mpg123 -q voice.mp3")
    os.remove("voice.mp3")

def run_jarvis():
    recognizer = sr.Recognizer()

    with sr.Microphone() as source:
        print("주변 소음 적응 중...")
        recognizer.adjust_for_ambient_noise(source, duration=1)
        speak("시스템이 준비되었습니다. 자비스라고 불러주세요.")

        while True:
            try:
                audio = recognizer.listen(source)
                wake_word = recognizer.recognize_google(audio, language='ko-KR')
                
                if "자비스" in wake_word:
                    speak("네, 부르셨나요? 명령을 말씀해주세요.")
                    
                    command_audio = recognizer.listen(source)
                    command = recognizer.recognize_google(command_audio, language='ko-KR')
                    print(f"인식된 명령: {command}")

                    if "터미널" in command:
                        speak("터미널을 실행합니다.")
                        subprocess.Popen(["gnome-terminal"])
                    
                    elif "준비" or "즌비" or "존비" in command:
                        speak("네, 로봇 제어 및 비전 시스템 환경을 순서대로 켜겠습니다.")
                        
                        my_ip = get_my_ip()
                        print(f"현재 IP 주소 감지됨: {my_ip}")
                        
                        subprocess.Popen(["gnome-terminal", "--", "bash", "-ic", "roboton; exec bash"])
                        time.sleep(0.5)
                        
                        subprocess.Popen(["gnome-terminal", "--", "bash", "-ic", "realsense; exec bash"])
                        time.sleep(0.5)
                        
                        subprocess.Popen(["gnome-terminal", "--", "bash", "-ic", "realsense-viewer; exec bash"])
                        time.sleep(0.5)
                        
                        # commander.py 실행 후 왼쪽 화면 꽉 차게 열기 (독립된 크롬 프로필 사용)
                        subprocess.Popen(["gnome-terminal", "--", "bash", "-ic", "cd /home/rokey/armbot_web && python3 commander.py; exec bash"])
                        time.sleep(3) 
                        subprocess.Popen(["google-chrome", f"--app=http://{my_ip}:5000", "--user-data-dir=/tmp/chrome_commander", "--window-position=0,0", "--window-size=960,1080"])
                        
                        # shooter.py 실행 후 오른쪽 화면 꽉 차게 열기 (독립된 크롬 프로필 사용)
                        subprocess.Popen(["gnome-terminal", "--", "bash", "-ic", "cd /home/rokey/armbot_web && python3 shooter.py; exec bash"])
                        time.sleep(3) 
                        subprocess.Popen(["google-chrome", f"--app=http://{my_ip}:5001", "--user-data-dir=/tmp/chrome_shooter", "--window-position=960,0", "--window-size=960,1080"])
                        
                        subprocess.Popen(["gnome-terminal", "--", "bash", "-ic", "source /opt/ros/humble/setup.bash && cd /home/rokey/cobot2_ws && source install/setup.bash && ros2 launch armbot armybot.launch.py; exec bash"])
                        
                        print("모든 터미널과 웹 화면 실행을 완료했습니다.")
                        speak("모든 시스템 준비가 완료되었습니다.")

                    else:
                        speak("아직 등록되지 않은 명령입니다.")
                        
            except sr.UnknownValueError:
                pass
            except sr.RequestError as e:
                print(f"음성 인식 서비스 에러: {e}")
                break

if __name__ == "__main__":
    run_jarvis()