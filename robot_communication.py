import socket
import time
#import rospy
#from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

class Server(object):
    PORT=5050
    def __init__(self):
        self.s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #self.pub=rospy.Publisher('slave_status',PoseStamped,queue_size=10)
        #self.sub=rospy.Subscriber("clean_robot_map",OccupancyGrid,self.mapCallback,queue_size=1000)
        #print(self.get_ip)
        self.map='1'+'\n'+'No_Data'+'\n'
        self.data=b''
        
    def get_ip(self):
        s1=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        try:
            #dosen't even have to be reachable
            s1.connect(('10.255.255.255',1))
            IP=s1.getsockname()[0]
        except:
            IP = '127.0.0.1'
        finally:
            s1.close()
        return IP
    
    def run(self):
        self.s.bind((str(self.get_ip()),5050))
        self.s.listen()
        conn, addr = self.s.accept()
        with conn:
            print('Connected by', addr)
            while True:
                self.data = conn.recv(1024)
                if not self.data:
                    break
                print(self.data)
                if str(self.data)[2:len(str(self.data))-1].split('\\n')[0]=='map':
                    conn.sendall(self.map.encode('utf-8'))
                    #conn.sendall(words.encode('utf-8'))
                elif str(self.data)[2:len(str(self.data))-1].split('\\n')[0]=='clean_region':
                    conn.sendall(str('1\nroger\n').encode('utf-8'))
               #time.sleep(0.01)
                #print(self.data)
        self.s.close()

    def mapCallback(self,msg):
        self.map='1'+'\n'+'No Data'+'\n'
        self.map=self.map+str(msg.info.width)+"\n"
        for i in range(0,msg.info.width*msg.info.width):
            self.map=self.map+chr(msg.data[i])
            if (i%64)==63:
                self.map=self.map+'\n'
        self.map=self.map+str(msg.info.pose.position.x)+' '+str(msg.info.pose.position.y)+'\n'

    def on_shutdown(self):
        rospy.logwarn("stop connect to client robot")
        

if __name__ == '__main__':
    #rospy.init_node('Server_node',anonymous= False) #if anonynous is true,Ros requires that each have a unique name
    node = Server()
    node.run()
	#rospy.on_shutdown(node.on_shutdown)
	#rospy.spin()