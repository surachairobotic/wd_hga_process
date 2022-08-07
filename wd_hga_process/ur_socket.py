import socket

class UR_SOCKET():
    def __init__(self, _ur_ip='192.168.12.100', _ur_port=30002, disableButton=True):
        self.ur_ip = _ur_ip
        self.ur_port = _ur_port
        self.connected = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    def init(self):
        x = self.sock.connect((self.ur_ip, self.ur_port))
        print('connect : ', str(x))
        self.connected = True

    def __del__(self):
        print('UR_SOCKET del')
        if self.connected:
            self.sock.close()
    
    def send(self, msg):
        msg += "\n"
        self.sock.send(msg.encode('utf-8'))
    def read(self):
        data = []
        while True:
            d = self.sock.recv(1024)
            print(d)
            if (len(d) < 1):
                break
            data.append(d)
        return data
    
    def stop(self):
        msg = 'stopj(2)'
        self.send(msg)
    
    def moveLine(self, p):
        pose = "{},{},{},{},{},{}".format(p[0], p[1], p[2], p[3], p[4], p[5])
        msg = 'movep(p[{}],a=0.1,v=0.15,r=0)'.format(pose)
        print(msg)
        self.send(msg)

    def moveJ(self, p):
        pose = "{},{},{},{},{},{}".format(p[0], p[1], p[2], p[3], p[4], p[5])
        msg = 'movej([{}],a=0.5,v=0.5,t=0,r=0)'.format(pose)
        print(msg)
        self.send(msg)
        
    '''
    def jointSpeed(self):
        msg = 'get_actual_joint_speeds()'
        self.send(msg)
        res = self.read()
        print(res)
    '''
