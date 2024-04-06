import sys  
import time
from PyQt5.QtWidgets import *  
from Ui_client import Ui_MainWindow
from PyQt5.QtCore import pyqtSignal,QObject
from PyQt5.QtGui import QImage,QPixmap,QIcon
import socket
import threading
import cv2
import numpy as np
import struct
class carinfo:
    linear_x = 0.0
    angual_z = 0.0
    odom_x = 0.0
    odom_y = 0.0
    odom_z = 0.0
    odom_w = 0.0
    yaw = 0
class client(QObject):
    show_vdieo = pyqtSignal(np.ndarray)
    show_data = pyqtSignal(float,str)
    show_carinfo = pyqtSignal(tuple)
    vdieo_value = b''
    car_value = b''
    aimdata =b''
    tempdata = b''
    def __init__(self,app):
       super(client,self).__init__()
       self.car = carinfo()
       self.app = app
       self.myapp = Ui_MainWindow()
       self.form = QMainWindow()
       self.form.move(640,400)
       self.myapp.setupUi(self.form)
       self.myapp.iptext.setText("127.0.0.1")
       self.myapp.porttext.setText("8899")
       self.myapp.label_20.setPixmap(QPixmap("12.png").scaled(69,38))
       self.form.setMaximumSize(1113,735)
       self.form.setMinimumSize(1113,735)
       self.form.setWindowTitle("client")
       self.form.show()
       msg_box = QMessageBox(QMessageBox.Information, "温馨提示", "请先点击连接服务器按扭，否则无法使用")
       msg_box.exec_()
        #绑定信号和曹
       self.bind_singals()
       
       self.vdieo_flag = 3
       self.carinfo_flag = 3
       self.flag = True
       self.tcp_flag = False
    #  信号和草
    def bind_singals(self):
        self.myapp.start_server.clicked.connect(self.connect_server)
        self.myapp.video_accpect.stateChanged.connect(lambda state: self.ifshowvideoinfo(1) if state >= 1 else self.ifshowvideoinfo(2))
        self.myapp.disconnect.clicked.connect(self.disconnect)
        self.myapp.speed_accpect.stateChanged.connect(lambda state: self.ifshowcarinfo(1) if state >=1 else self.ifshowcarinfo(2))
        self.myapp.sendbtn.clicked.connect(self.sendtext_toserver)
        self.myapp.up.clicked.connect(lambda: self.sendlinearspeed_toserver(0.2))
        self.myapp.down.clicked.connect(lambda: self.sendlinearspeed_toserver(-0.2))
        self.myapp.right.clicked.connect(lambda: self.sendangualspeed_toserver(-0.2))
        self.myapp.left.clicked.connect(lambda: self.sendangualspeed_toserver(0.2))
        self.myapp.stop.clicked.connect(lambda: self.sendangualspeed_toserver(0))
        self.myapp.stop.clicked.connect( lambda: self.sendlinearspeed_toserver(0))
        self.myapp.send_loc.clicked.connect(lambda: self.send_loc(float(self.myapp.x_aim.text()),float(self.myapp.y_aim.text())))
        self.show_vdieo.connect(self.show_fream)
        self.show_data.connect(self.append_data)
        self.show_carinfo.connect(self.append_carinfo)
    #显示小车参数
    def append_carinfo(self,data):
        self.myapp.linea_text.setText(str(data[0])+"m/s")
        self.myapp.angual_text.setText(str(data[1])+"m/s")
        self.myapp.x_text.setText(str(data[2])+"m")
        self.myapp.y_text.setText(str(data[3])+"m")
        self.myapp.z_text.setText(str(data[4])+"m")
        self.myapp.w_text.setText(str(data[5])+"m")
        self.myapp.yaw_text.setText(str(data[6]))
    #是否显示小车数据接收状态
    def ifshowcarinfo(self,value):
        self.carinfo_flag = value
        if value ==1:self.car = True
    #是否显示视频数据接收状态
    def ifshowvideoinfo(self,value):
        self.vdieo_flag = value
        if value ==1:self.video = True
    #添加数据
    def append_data(self,times,data):
        self.myapp.commuaction.append("["+str(times)+"]"+data)
    # 向服务器发送文本
    def sendtext_toserver(self):
        self.client_socket.sendall((0xDD).to_bytes(1,"little")+self.myapp.text.toPlainText().encode(encoding="utf-8"))
        self.show_data.emit(time.time(),f"客户端说：{self.myapp.text.toPlainText()}")
        self.myapp.text.clear()
    # 向服务器发送线速度
    def sendlinearspeed_toserver(self,value):
        self.client_socket.sendall((0xEE).to_bytes(1,"little")+str(value).encode(encoding="utf-8"))
        self.show_data.emit(time.time(),f"小车线速度:{str(value)}")
     # 向服务器发送角速度
    # 向服务器发送角速度
    def sendangualspeed_toserver(self,value):
        self.client_socket.sendall((0xFF).to_bytes(1,"little")+str(value).encode(encoding="utf-8"))
        self.show_data.emit(time.time(),f"小车角速度:{str(value)}")
    #发送位置数据
    def send_loc(self,x,y):
        loc_val = struct.pack("ff",x,y)
        self.client_socket.sendall((0xFD).to_bytes(1,"little")+loc_val)
                                  
        self.show_data.emit(time.time(),f"x坐标: {self.myapp.x_aim.text()} y坐标: {self.myapp.y_aim.text()}")
    #连接服务器
    def connect_server(self):
        self.client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.client_socket.connect((self.myapp.iptext.text(),int(self.myapp.porttext.text())))
        self.show_data.emit(time.time(),":服务器连接成功")
        #通知服务器可以发送数据
        self.client_socket.sendall((0xCC).to_bytes(1,"little"))
        #启动接收数据的线程，避免阻塞主线程
        self.th = threading.Thread(target=self.recv)
        self.th.start()     
    #接收缓冲区数据
    def recv(self):
        while self.flag == True:
            data = self.client_socket.recv(88888) #收到数据，字节长度最大888888
            #收到服务器断开消息，break
            if data == b"exit": 
                self.show_data.emit(time.time(),":服务器断开连接")
                break
                
            if data[0:2] == 0x55aa.to_bytes(2,"little") or self.tcp_flag: #判断包头
                self.tcp_flag = True #第一次发送完整数据标志位
                if data[-2:] == 0x55cc.to_bytes(2,"little"): #判断包尾
                    self.aimdata = self.tempdata + data #将最后一次接收到的数据加上
                    longth = len(self.aimdata) #打印数据总长
                    video_length = longth- 36 #计算视频字节大小
                    # print(len(self.aimdata))
                    self.tcp_flag = False # 第一次发送完了完整数据后，标志位置0
                    if self.aimdata[2:6] == len(self.aimdata).to_bytes(4,"little"): #判断数据总长是否正确
                        dataval = struct.unpack('fffffff',self.aimdata[6:34]) #除去包头后的28位字节为小车参数
                        if self.carinfo_flag == 1 :
                            if self.car == True:
                                self.show_data.emit(time.time(),"开始接收小车参数") 
                                self.car = False #保证上述信号只发一次
                            self.show_carinfo.emit(dataval) #发送信号到主线程显示

                        elif self.carinfo_flag == 2 :
                            self.show_data.emit(time.time(),"停止接收小车参数")
                            self.carinfo_flag = 3
                             
                        self.vdieo_value = self.aimdata[34:34+video_length] #小车参数后的字节开始数video_length个字节为图像数据
                        nparry = np.frombuffer(self.vdieo_value,dtype="uint8")
                        fream = cv2.imdecode(nparry,cv2.IMREAD_COLOR) #视频帧解码

                        if fream is not None and self.vdieo_flag == 1:
                            if self.video  == True:
                                self.show_data.emit(time.time(),"开始接收视频参数")
                                self.video = False
                            self.show_vdieo.emit(fream) #将视频帧发送主线程显示

                        elif self.vdieo_flag == 2:
                            self.show_data.emit(time.time(),"停止接收视频参数")
                            self.vdieo_flag = 3 #保证上述信号只发一次
                    #第一次数据接收完成后字节段清空
                    self.tempdata = b''
                    self.aimdata = b''
                else:
                    self.tempdata += data #半包现象解决核心，没有到包尾就将字节拼接                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
    # 图像帧处理
    def show_fream(self,value):
         value = cv2.cvtColor(value,cv2.COLOR_BGR2RGB)
         height, width, channels = value.shape
        # 转换成QImage在 ui上显示
         images = QImage(value.data, width, height, width * channels, QImage.Format_RGB888)
         self.myapp.video.setPixmap(QPixmap.fromImage(images))
    #断开连接
    def disconnect(self):
        self.flag = False
        self.client_socket.sendall("exit".encode(encoding="utf-8"))
        self.show_data.emit(time.time(),":客户端断开连接")
        time.sleep(0.5)
        self.client_socket.close()
#主函数
if __name__ == "__main__":
     app  = QApplication(sys.argv)
     testr = client(app)
     testr.app.exec_()  