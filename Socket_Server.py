import socket
import time
import clrobot_self as self

HOST = self.get_ip()  # Standard loopback interface address (localhost)
PORT = 5050        # Port to listen on (non-privileged ports are > 1023)


print(HOST)
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
	s.bind((HOST, PORT))
	s.listen()
	conn, addr = s.accept()
	with conn:
		print('Connected by', addr)
		time_now=time.time()
		while (time.time()-time_now)<60:
			data = conn.recv(1024)
			if not data:
				break
			else:
				print(data)
			#conn.sendall(data)
