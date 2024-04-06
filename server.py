import socket
import numpy as py 
import cv2
import rospy
import struct
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import threading
from PyQt5.QtCore import pyqtSignal,QObject
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
class carinfo:
    linear_x = 0.0
    angular_z = 0.0
    odom_x = 0.0
    odom_y = 0.0
    odom_z = 0.0
    odom_w = 0.0
    yaw = 0

    def encode(self):
        return struct.pack('fffffff',self.linear_x,self.angular_z,self.odom_x,self.odom_y,self.odom_z,self.odom_w
                           ,self.yaw
                           )
class server(QObject):
    cap = cv2.VideoCapture(6)
    car = carinfo()
    def __init__(self):
        self.flag =True
        rospy.init_node("server")
        rospy.Subscriber("cmd_vel",Twist,self.speed_callback)
        rospy.Subscriber("odom",Odometry,self.odom_callback)
        rospy.Subscriber("yaw",Float32,self.yaw_callback)
        self.cmd_pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
        self.video_th2 = threading.Thread(target=self.ros_loop) 
        self.video_th2.start()
        self.connect_server()
    #发送目标点
    def pub_goals(self,x,y):
        ac = SimpleActionClient("move_base",MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        if not ac.wait_for_server(rospy.Duration(30)):
            rospy.loginfo("Can't connected to move base server")
            rospy.signal_shutdown("Unable to connect to move base server")
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 1.0
        goal.target_pose.pose.orientation.z = 0
        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.w = 1.0
        
        ac.send_goal(goal)
        print("目标点已发送",x,y)
    #连接服务器
    def connect_server(self):
        self.ser_soc = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.ser_soc.bind(("192.168.1.107",8899))
        self.ser_soc.listen(5)
        twist = Twist()
        print("服务器监听中")
        self.socketvalue, addr= self.ser_soc.accept()
        self.socketvalue.settimeout(3600) #设置超时时间1小时
        print("连接到客户端",addr)
        
        while True:            
            data = self.socketvalue.recv(1024)
            #0xCC 发送整包数据开始信号
            if data[0] == 0xCC:
                self.video_th = threading.Thread(target=self.send_fream) 
                self.video_th.start()
            #0xDD 文本数据
            elif data[0] == 0xDD:
                print("客户端说："+data[1:].decode(encoding="utf-8"))
            #0xEE 线速度数据
            elif data[0] == 0xEE:
                twist.linear.x = float(data[1:].decode(encoding="utf-8"))
                twist.angular.z = 0
                print("线速度",twist.linear.x)
                self.cmd_pub.publish(twist)
            # 0xFF 角速度数据
            elif data[0] == 0xFF:
                twist.linear.x = 0
                twist.angular.z = float(data[1:].decode(encoding="utf-8"))
                print("角速度",twist.angular.z)
                self.cmd_pub.publish(twist)
            #0xFE 位置数据
            elif data[0]  == 0xFD:
                loc = struct.unpack("ff",data[1:])
                # print(float(loc[0]))
                # print(float(loc[1]))
                self.pub_goals(float(loc[0]),float(loc[1]))
            elif data == b"exit":
                print("客户端断开连接,在时间",time.time())
                break
        self.flag = False
    #法数据函数
    def send_fream(self):
        while self.flag == True:
            try:
                rat, fream = self.cap.read()
                fream =  cv2.resize(fream,(382,314)) #设定图像大小
                info = cv2.imencode('.jpg',fream)[1] #视频帧编码
                self.video_data = py.array(info).tobytes()
                car_info = self.car.encode()
                #数据包格式
                # 0x55aa  + 整包长度     +  小车参数  +    视频      + 0x55cc
                #  4bytes + 36+x个bytes +  28bytes  + 不定长bytes  + 4bytes
                valuedata = (0x55aa).to_bytes(2,"little")+(len(car_info+self.video_data)+8).to_bytes(4,"little")+self.car.encode()+self.video_data+(0x55cc).to_bytes(2,"little")
                self.socketvalue.sendall(valuedata)
                # print(len(valuedata))
                # break
                # print(valuedata)
            except:
                break
        self.ser_soc.close() 
        self.socketvalue.close()
        self.cap.release()
        cv2.destroyAllWindows
    # 新线程负责循环更新ros话题通信的缓冲区
    def ros_loop(self):
        rospy.spin()
        if self.flag == False:
            return 
    # 里程计订阅
    def odom_callback(self,data):
        self.car.odom_x= data.pose.pose.position.x
        self.car.odom_y = data.pose.pose.position.y
        self.car.odom_z = data.pose.pose.position.z
        self.car.odom_w = data.pose.pose.orientation.w     
    #速度订阅
    def speed_callback(self,data):
        self.car.linear_x = data.linear.x
        self.car.angular_z = data.angular.z
    # 偏航角订阅
    def yaw_callback(self,data):
        self.car.yaw = data.data
# 主函数
if __name__ == "__main__":
    ser = server()
        