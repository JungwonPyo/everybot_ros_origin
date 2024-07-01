#!/usr/bin/env python
# -*- coding: utf-8 -*-

###########################################################################
# control 디버그용 제어 리모콘 프로그램
# 제어 모드는 2가지 존재
# 1. 속도 모드
# 영문 방향키를 사용하여 선속도와 각속도로
# 제어 명령을 담은 ros 메세지를 publish 함
#
# 2. 제어 모드
# 영문 방향키를 사용하여 전 후 좌 우 정지에 대한
# 제어 명령을 담은 ros 메세지를 publish 함.
# 1~9 숫자키를 사용하여 제어 명령을 담은 ros 메세지를 publish 함.
# 
# 23.01.31 jspark
###########################################################################
import color as c
import rospy
import os
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
import sys, select, termios, tty

funckey_dict = {
    'f1':"\x1bOP",
    'f2':"\x1bOQ",
    'f3':"\x1bOR",
    'f4':"\x1bOS",
}

def rad2deg(rad):
    return rad*180/3.141592
def deg2rad(deg):
    return deg*3.141592/180
class Remocon:
    def __init__(self):
        self.__control_type = "direction"
        self.__command_type = "기본"
        self.__lastKey = "1"
        self.__settings = termios.tcgetattr(sys.stdin)
        rospy.init_node('remote_key_node')
        self.__pub_velocity = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.__pub_target = rospy.Publisher("/cmd_target", Vector3, queue_size=5)
        self.__pub_ctrl = rospy.Publisher("/cmd_dir", String, queue_size=5)

        self.__velMsg = Twist() # 단위 m/s, rad/s
        self.__targetMsg = Vector3() # 단위 m, m, rad
        self.__keyMsg = String()

        # 속도 제어용 변수
        self.__linear_velocity = 0.0        # 선속도 (단위 m/s)
        self.__angular_velocity = 0.0       # 각속도 (단위 rad/s)
        self.__DELTA_LINEAR = 0.05          # 선속도 변위 (±50 mm/s)
        self.__DELTA_ANGULAR = 0.0174533*50 # 각속도 변위 (±50 deg/s)
        self.__LIMIT_LINEAR = 0.33          # 선속도 제한
        self.__LIMIT_ANGULAR = 0.0174533*350# 각속도 제한

    def __publishControlTopic(self):
        self.__velMsg.linear.x = self.__linear_velocity
        self.__velMsg.angular.z = self.__angular_velocity
        self.__pub_velocity.publish(self.__velMsg)

    def __publishCommandTopic(self, command):
        self.__keyMsg.data = command
        self.__pub_ctrl.publish(self.__keyMsg)

    def __publishTargetTopic(self):
        self.__targetMsg.z = deg2rad(self.__targetMsg.z)
        self.__pub_target.publish(self.__targetMsg)

    def __minmaxVelocity(self, linear, angular):
        if linear > self.__LIMIT_LINEAR:
            linear = self.__LIMIT_LINEAR
        if linear < self.__LIMIT_LINEAR*(-1):
            linear = self.__LIMIT_LINEAR*(-1)
        
        if angular > self.__LIMIT_ANGULAR:
            angular = self.__LIMIT_ANGULAR
        if angular < self.__LIMIT_ANGULAR*(-1):
            angular = self.__LIMIT_ANGULAR*(-1)
        
        return linear, angular

    def printMenu(self):
        os.system("clear")
        # time.sleep(0.1)
        print("")
        # print("------- Control Your Robot -------")
        self.printControl()
        if self.__control_type != "target": # 타겟 제어일때는 명령키 사용하지 않음
            self.printCommand()

    def printControl(self):
        print(c.BPurple("========================================"))
        print( c.BWhite("            ( F1 )  제어 모드            "))
        if (self.__control_type == "direction"):
            print(c.BGreen("         ( ")+c.On_Green("방향")+c.BGreen(" / 속도 / 타겟 )"))
            print(c.Purple("----------------------------------------"))
            print(c.BWhite("     w     ")+"|")
            print(c.BWhite("   a s d   ")+"|")
            print(c.BWhite("     x     ")+"|")
            print(c.Purple("----------------------------------------"))
        elif (self.__control_type == "velocity"):
            print(c.BGreen("         ( 방향 / "+c.On_Green("속도")+c.BGreen(" / 타겟 )")))
            print(c.BWhite("     w     ")+"|")
            print(c.BWhite("   a s d   ")+"|"+" 선속도: \033[1;32m{:.0f}\033[0m mm/s".format(self.__linear_velocity*1000))
            print(c.BWhite("     x     ")+"|"+" 각속도: \033[1;32m{:.0f}\033[0m deg/s".format(rad2deg(self.__angular_velocity)))
            print(c.Purple("----------------------------------------"))
        else:
            print(c.BGreen("         ( 방향 / 속도 / "+c.On_Green("타겟")+c.BGreen(" )")))
            print(c.BWhite("     w     ")+"|")
            print(c.BWhite("   a s d   ")+"|"+" \t좌표 (\033[1;32m{:.2f}\033[0m m, \033[1;32m{:.2f}\033[0m m) ".format(self.__targetMsg.x, self.__targetMsg.y))
            print(c.BWhite("     x     ")+"|"+" \t헤딩 \033[1;32m{:.1f}\033[0m deg".format(self.__targetMsg.z))
            print(c.Purple("----------------------------------------"))
            print(" \033[1;32mq\033[0m 를 입력하여 타겟 제어모드 종료")
            print(c.Purple("----------------------------------------"))

    def printCommand(self):
        print(c.BPurple("========================================"))
        print( c.BWhite("            ( F2 )  명령 모드            "))
        if (self.__command_type == "기본"):
            print(c.BGreen("         ( ")+c.On_Green("기본")+c.BGreen(" / 패턴 / empty )"))
            print(c.Purple("----------------------------------------"))
            print("  1 | 자동청소 \t\t   p | 전원 OFF")
            print("  2 | 도킹     \t\t   i | IMU 초기화")
            print("  3 | 탐색     \t\t   u | 볼륨조절")
            print("  4 | 앱등록   \t\t   y | 물공급 조절")
            print("  5 | 대기모드 \t\t   t | TOF 초기화")
            print("  6 | 틸팅 UP \t\t   f | 슬램 ON")
            print("  7 | 틸팅 DOWN\t\t   g | 슬램 OFF")
            print("  8 | 잔수제거 \t\t   h | 지도저장")
            print("  9 | 걸레건조 \t\t   j | 지도삭제")
            print("  k | FW-UPDATE\t\t   l | FW-RECOVERY")
            print("  c | 예약청소 \t\t   b | 방해금지")
            print("  n | 공장초기화\t\t  m | MCU재부팅")
            print("  r | AP재부팅")
            print(c.Purple("----------------------------------------"))
        elif (self.__command_type == "패턴"):
            print(c.BGreen("         ( 기본 / "+c.On_Green("패턴")+c.BGreen(" / empty )")))
            print(c.Purple("----------------------------------------"))
            print("  1 | 1번 패턴")
            print("  2 | 2번 패턴")
            print("  3 | 3번 패턴")
            print("  4 | 4번 패턴")
            print("  5 | 5번 패턴")
            print("  6 | 6번 패턴")
            print("  7 | 7번 패턴")
            print("  8 | 8번 패턴")
            print("  9 | 9번 패턴")
            print(c.Purple("----------------------------------------"))
        elif (self.__command_type == "empty"):
            print(c.BGreen("         ( 기본 / 패턴 / "+c.On_Green("empty")+c.BGreen(" )")))
            print(c.Purple("----------------------------------------"))
            print(c.Purple("----------------------------------------"))
        else:
            print(c.BGreen("   ( 기본 / 패턴 / empty "))
            print(c.BRed(" 없는 타입이에요."))

    def __getRawKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b':
                key += sys.stdin.read(2)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.__settings)
        return key

    def getKey(self):
        key_string = self.__getRawKey()
        if (key_string == '\x03'):
            return "exit"
        elif (key_string == funckey_dict['f1']):
            return "f1"
        elif (key_string == funckey_dict['f2']):
            return "f2"
        elif (key_string == funckey_dict['f3']):
            return "f3"
        elif (key_string == funckey_dict['f4']):
            return "f4"
        else:
            return key_string
    
    def isTargetControlMode(self):
        return self.__control_type=="target"

    def __setTargetXPose(self, x):
        self.__targetMsg.x = float(x)

    def __setTargetYPose(self, y):
        self.__targetMsg.y = float(y)

    def __setTargetThetaPose(self, theta):
        self.__targetMsg.z = float(theta)

    def __clearTargetPose(self):
        self.__targetMsg.x = 0.0
        self.__targetMsg.y = 0.0
        self.__targetMsg.z = 0.0
        pass

    def controlKeyList(self):
        return ['w', 'a', 's', 'd', 'x']

    def modeChangeKeyList(self):
        return ['f1', 'f2']

    def isControlKey(self, key):
        return True if(key=='w' or key=='a' or key=='s' or key=='d' or key=='x') else False

    def modeChangeProc(self, key):
        if key=='f1': # 제어키 변경
            if(self.__control_type=='direction'):
                self.__control_type='velocity'
                self.__linear_velocity = 0.0
                self.__angular_velocity = 0.0
            elif(self.__control_type=='velocity'):
                self.__control_type='target'
                self.__linear_velocity = 0.0
                self.__angular_velocity = 0.0
            else:
                self.__control_type='direction'
                self.__linear_velocity = 0.0
                self.__angular_velocity = 0.0

        elif key == 'f2': # 명령키 변경
            if(self.__command_type=='기본'):
                self.__command_type='패턴'
            elif(self.__command_type=='패턴'):
                self.__command_type='empty'
            else:
                self.__command_type='기본'
        else:
            print("modeChangeProc() 잘못된 key")

    def controlProc(self, key):
        if self.__control_type == 'direction':
            self.__controlDirectionProc(key)
        else:
            self.__controlVelocityProc(key)
            self.__publishControlTopic()
    
    def __controlDirectionProc(self, key):
        if key == 'w': # 전진
            self.__publishCommandTopic('movego')
        elif key == 'a': # 왼쪽
            self.__publishCommandTopic('turnleft')
        elif key == 's': # 정지
            self.__publishCommandTopic('movestop')
        elif key == 'd': # 오른쪽
            self.__publishCommandTopic('turnright')
        elif key == 'x': # 후진
            self.__publishCommandTopic('moveback')

    def __controlVelocityProc(self, key):
        if key == 'w':
            self.__linear_velocity += self.__DELTA_LINEAR
        elif key == 'x':
            self.__linear_velocity -= self.__DELTA_LINEAR
        elif key == 'a':
            self.__angular_velocity += self.__DELTA_ANGULAR
        elif key == 'd':
            self.__angular_velocity -= self.__DELTA_ANGULAR
        elif key == 's':
            self.__linear_velocity = 0.0
            self.__angular_velocity = 0.0

        self.__linear_velocity, self.__angular_velocity = self.__minmaxVelocity(self.__linear_velocity, self.__angular_velocity)

    def targetProc(self):
        while(1):
            try:
                key = raw_input("x 값을 입력하세요: ")
                if( key == 'q' ):
                    rc.__clearTargetPose()
                    rc.commandProc('f1')
                    rc.printMenu()
                    return False
                rc.__setTargetXPose( float(key) )
                rc.printMenu()
                
                key = raw_input("y 값을 입력하세요: ")
                if( key == 'q' ):
                    rc.__clearTargetPose()
                    rc.commandProc('f1')
                    rc.printMenu()
                    return False
                rc.__setTargetYPose( float(key) )
                rc.printMenu()
                
                key = raw_input("θ 값을 입력하세요: ")
                if( key == 'q' ):
                    rc.__clearTargetPose()
                    rc.commandProc('f1')
                    rc.printMenu()
                    return False
                rc.__setTargetThetaPose( float(key) )
                rc.printMenu()

                rc.__publishTargetTopic()
                rc.__clearTargetPose()
                return True
            except ValueError:
                print("유효한 실수 값을 입력하세요.")

    def commandProc(self, key):
        if key=='f1': # 제어키 변경
            if(self.__control_type=='direction'):
                self.__control_type='velocity'
                self.__linear_velocity = 0.0
                self.__angular_velocity = 0.0
            elif(self.__control_type=='velocity'):
                self.__control_type='target'
                self.__linear_velocity = 0.0
                self.__angular_velocity = 0.0
            else:
                self.__control_type='direction'
                self.__linear_velocity = 0.0
                self.__angular_velocity = 0.0

        elif key == 'f2': # 명령키 변경
            if(self.__command_type=='기본'):
                self.__command_type='type2'
            elif(self.__command_type=='type2'):
                self.__command_type='pattern'
            else:
                self.__command_type='기본'

        elif self.__command_type=='기본':
            if key == '1': # 자동청소
                self.__publishCommandTopic('CLEAN')
            elif key == '2':
                self.__publishCommandTopic('HOME')
            elif key == '3':
                self.__publishCommandTopic('EXPLORE')
            elif key == '4':
                self.__publishCommandTopic('WIFI')
            elif key == '5':
                self.__publishCommandTopic('STOP')
            elif key == '6':
                self.__publishCommandTopic('TILTING_UP')
            elif key == '7':
                self.__publishCommandTopic('TILTING_DONW')
            elif key == '8':
                self.__publishCommandTopic('DRAIN_WATER')
            elif key == '9':
                self.__publishCommandTopic('DRY_MOP')
            elif key == 'p': 
                self.__publishCommandTopic('POWER_OFF') 
            elif key == 'i':
                self.__publishCommandTopic('IMU_INIT')          
            elif key == 'u':
                self.__publishCommandTopic('VOLUME')
            elif key == 'y': 
                self.__publishCommandTopic('WATER')
            elif key == 't': 
                self.__publishCommandTopic('TOF_INIT')    
            elif key == 'f': 
                self.__publishCommandTopic('SLAM_ON')
            elif key == 'g': 
                self.__publishCommandTopic('SLAM_OFF')    
            elif key == 'h': 
                self.__publishCommandTopic('SAVE_MAP')
            elif key == 'j':
                self.__publishCommandTopic('DELETE_MAP')
            elif key == 'k': 
                self.__publishCommandTopic('FW_UPDATE')
            elif key == 'l':
                self.__publishCommandTopic('FW_RECOVERY')
            elif key == 'c': 
                self.__publishCommandTopic('RESERVATION_CLEAN')
            elif key == 'v': 
                self.__publishCommandTopic('RESERVATION_STOP')
            elif key == 'b': 
                self.__publishCommandTopic('INIT_USERSET')
            elif key == 'n': 
                self.__publishCommandTopic('FACTORY_RESET')
            elif key == 'm': 
                self.__publishCommandTopic('MCU_RESET')    
            elif key == 'r': 
                self.__publishCommandTopic('AP_RESET')           
        elif self.__command_type=='패턴':
            if key == '1':
                self.__publishCommandTopic('PATTERN_1')
            elif key == '2':
                self.__publishCommandTopic('PATTERN_2')
            elif key == '3':
                self.__publishCommandTopic('PATTERN_3')
            elif key == '4':
                self.__publishCommandTopic('PATTERN_4')
            elif key == '5':
                self.__publishCommandTopic('PATTERN_5')
            elif key == '6':
                self.__publishCommandTopic('PATTERN_6')
            elif key == '7':
                self.__publishCommandTopic('PATTERN_7')
            elif key == '8':
                self.__publishCommandTopic('PATTERN_8')
            elif key == '9':
                self.__publishCommandTopic('PATTERN_9')

if __name__=="__main__":
    rc = Remocon()
    rc.printMenu()

    while(1):
        if(rc.isTargetControlMode()):
            while True:
                if (rc.targetProc() == False):
                    break

                choice = raw_input("목표 좌표를 추가로 입력하시겠습니까? (y/n): ")
                if choice.lower() != 'y':
                    rc.commandProc('f1')
                    rc.printMenu()
                    break
        else:
            key = rc.getKey()
            if (key==''):
                pass
            elif (key=="exit"):
                break
            else:
                if (key in rc.modeChangeKeyList()):
                    rc.modeChangeProc(key)
                    rc.printMenu()
                # if (rc.isControlKey(key)):
                elif (key in rc.controlKeyList()):
                    rc.controlProc(key)
                    rc.printMenu()
                    print("\n 입력 키: " + key)
                else:
                    rc.commandProc(key)
                    rc.printMenu()
                    if( key != 'f1'):
                        print("\n 입력 키: " + key)
    exit(-1)
