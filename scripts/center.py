import socket
import datetime
import sys
import trollius
from trollius import From
import pygazebo
import pygazebo.msg.gz_string_pb2


server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
server_socket.bind(('127.0.0.1',8080))

server_socket.listen(0)
conn, addr = server_socket.accept()

def truck_action(loop,conn,data):
	if data[0]=="arrive":
		f = open("/home/manros/catkin_ws/src/capstone/scripts/data/enter_info.txt", 'r')
		while True:
			line = f.readline()
			if not line: break
			if (line.find(data[1]))!=-1:
				blockbar_action()
				loop.run_until_complete(blockbar_action()) # send topic to gazebo


	elif data[0]=="finish":
		f = open("/home/manros/catkin_ws/src/capstone/scripts/data/out_info.txt", 'a')
		data.append(datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
		f.write(str(data)+"\n")
		loop = trollius.get_event_loop()
		loop.run_until_complete(blockbar_action()) # send topic to gazebo


@trollius.coroutine
def blockbar_action():
	manager = yield From(pygazebo.connect())
	publisher = yield From(manager.advertise('/gazebo/vrc_task_1/blockingbar','gazebo.msgs.GzString'))

	message = pygazebo.msg.gz_string_pb2.GzString()
	message.data = "-1"

	while True:
		yield From(publisher.publish(message))
		yield From(trollius.sleep(1.0))
		print message
		

if __name__ == "__main__":
	while True:
		try:
			loop = trollius.get_event_loop()


			data = conn.recv(65535)
			print "recieve data : %s"% data.decode()

			data = data.split(" ")

			truck_action(loop,conn,data)

		except KeyboardInterrupt:
			conn.close()
