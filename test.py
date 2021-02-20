import RPi.GPIO as GPIO
import time
import socket
import clrobot_self as self
import board
import sys
from adafruit_motorkit import MotorKit
HOST=self.get_ip()
PORT=5050

print(HOST)

"""GPIO.setmode(GPIO.BCM)#GPIO has two mode respectively GPIO.BCM and GPIO.BOARD
mode=GPIO.getmode()
print(mode)
GPIO.setwarnings(False)
GPIO.setup(4,GPIO.OUT,initial=GPIO.LOW)#設定初始值是低電位

time_start=time.time()
time_end=time.time()
i=0
while (time_end-time_start)<= 10:
	time_now=time.time()
	if (time_now-time_end)>=1:
		time_end=time_now
		i=i+1
		print(time_end-time_start)
	if (i%2)==0:
		GPIO.output(4,GPIO.HIGH)
	else:
		GPIO.output(4,GPIO.LOW)
GPIO.cleanup()#釋放資源"""
kit=MotorKit(i2c=board.I2C())
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s: #AF_INET represent server vs server connect
															 #SOCK_STREAM is TCP/IP
	s.bind((HOST,PORT))
	s.listen()
	conn, addr =s.accept()
	with conn:
		print('Connected by',addr)
		time_now=time.time()
		while True:
			data=conn.recv(1024)
			if not data:
				break
			else:
				print(data)
			if data!=(b'no asign'):
				direct=str(data)[2:len(str(data))-1]
			if direct=="Stop":
				kit.motor1.throttle =0
				kit.motor2.throttle =0
			elif direct=="forward" :
				kit.motor1.throttle=0.3
				kit.motor2.throttle =0.3
			elif direct=="backward":
				kit.motor1.throttle=-0.3
				kit.motor2.throttle =-0.3								
			elif direct=="right":
				kit.motor1.throttle=0
				kit.motor2.throttle =0.3								
			elif direct=="left":
				kit.motor1.throttle=0.3
				kit.motor2.throttle =0
			else:
				kit.motor1.throttle =0
				kit.motor2.throttle =0				
kit.motor1.throttle=0
