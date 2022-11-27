#Library importation
import socket

#Socket setings
HOST="192.168.12.100" #replace by the IP address of the UR robot
PORT=63352 #PORT used by robotiq gripper

#Socket communication
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    #open the socket
    s.connect((HOST, PORT))
    s.sendall(b'SET POS 81\n')
    data = s.recv(2**10)

#Print finger position
#Gripper finger position is between 0 (Full open) and 255 (Full close)
print('Gripper finger position is: ', data)


