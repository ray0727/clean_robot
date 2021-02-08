import socket


class Remote:
	Direction={
		"forward":1,
		"backward":2,
		"left":3,
		"right":4,
		"stop":0
    }

	def __init__(self, state=0):
		self.state=state
	def get_state(self):
		return self.state
def get_ip():
	s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
	try:
		#dosen't even have to be reachable
		s.connect(('10.255.255.255',1))
		IP=s.getsockname()[0]
	except:
		IP = '127.0.0.1'
	finally:
		s.close()
	return IP
