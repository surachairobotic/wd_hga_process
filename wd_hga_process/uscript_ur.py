import time, keyboard
from ur_socket import *

# Gripper Variables
ACT = "ACT"
GTO = "GTO"
ATR = "ATR"
ARD = "ARD"
FOR = "FOR"
SPE = "SPE"
OBJ = "OBJ"
STA = "STA"
FLT = "FLT"
POS = "POS"

SOCKET_HOST = "127.0.0.1"
SOCKET_PORT = 63352
SOCKET_NAME = "gripper_socket"

def main(args=None):

    ur = UR_SOCKET()
    ur.init()
    ur.stop()    
    
    jj = [  [1.6100164651870728, -1.64317049602651, 2.4955900351153772, -3.99554242710256, -1.6224005858050745, 3.14430570602417],
            [4.368937015533447, -1.2535660129836579, 2.224023167287008, -4.138756414453024, -1.2165988127337855, 3.145020008087158],
            [4.37146520614624, -0.012463168506958056, 1.7640226523028772, -4.91780223468923, -1.2204092184649866, 3.1394753456115723]
         ]
    pp = [  [ 0.14075575758431658, -0.5505126556505817,  0.3912572106874151,  1.5674877209203752, -0.00001, -0.00001],
            [-0.01376703336070136,  0.7046249426517024,  0.3499899351397665,  0.0305088624493274, -2.19006, -2.24545],
            [-0.013694179808458936, 0.7046214638825812, -0.11002220052983638, 0.030255677140335162, -2.190209656127298, -2.2453879746576333]
            
         ]
    
    '''
    end = pp[2]
    end[0] += 0.1
    end[1] += 0.23
    ur.moveLine(end)
    
    exit()
    '''
    
    fw = 1
    if fw == 1:
        ur.moveJ(jj[0])
        input()
        ur.moveJ(jj[1])
        input()
        ur.moveLine(pp[2])
        input()
        end = pp[2]
        end[0] += 0.1
        end[1] += 0.23
        ur.moveLine(end)
        input() # G-
        ur.moveLine(pp[1])        
        input()
        ur.moveJ(jj[0])
        # G+
    elif fw == 2:
        ur.moveJ(jj[2])
        input()
        ur.moveLine(pp[1])
        input()
        ur.moveJ(jj[0])
    elif fw == 3:
        ur.moveLine(pp[1])
        input()
        ur.moveJ(jj[0])    
    #print(ur.read())
    exit()


    HOST = "192.168.12.100"
    PORT = 30002
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    
    d = [90,-90,140,-237,-90,180]
    j = deg2rad(d)
    print(d)
    print(j)

    '''
    def _set_gripper_activate(self):
        self._socket_set_var(GTO, 1, self.socket_name)

    msg = "socket_set_var(\"{}\",{},\"{}\")".format(var, value, socket_name)  # noqa
    '''

    #cmd = "set_digital_out(0,False)" + "\n"


    var = GTO
    value = 1
    socket_name = SOCKET_NAME
    socket_host = SOCKET_HOST
    socket_port = SOCKET_PORT
    msg = "\tsocket_close(\"{}\")".format(socket_name)

    msg = msg + "\n\t" + "socket_open(\"{}\",{},\"{}\")".format(socket_host,
                                                 socket_port,
                                                 socket_name)

    msg = msg + "\n\t" + "socket_set_var(\"{}\",{},\"{}\")".format(var, value, socket_name)  # noqa
    

    cmd = [ "rq_activate()\n",
    "rq_reset()\n",
    "movej([1.57,-1.57,2.45,-4.13,-1.57,3.14],a=1.0,v=0.5,t=0,r=0)" + "\n",
    "movej([1.0,-1.57,2.45,-4.13,-1.57,3.14],a=1.0,v=0.5,t=0,r=0)" + "\n",
    ]
    indx = 0
    #cmd = "get_actual_joint_positions()" + "\n"    

    myprog = """def myProg():{}\nend""".format(msg)

    
    loop = True
    while loop:
        if keyboard.is_pressed('q'):
            break
        elif keyboard.is_pressed('w'):
            s.send(cmd[indx].encode('utf-8'))
            data = s.recv(1024)
            
            indx = (indx+1) % 2
        time.sleep(0.05)
    
    if not loop:
        s.send(cmd[0].encode('utf-8'))
        data = s.recv(1024)
        print('Recv : ', str(data))
        time.sleep(0.1)
        s.send(cmd[1].encode('utf-8'))
        data = s.recv(1024)
        print('Recv : ', str(data))
    
    s.close()

    '''    
    print('Recv : ', str(data))
    print('-----')
    for x in data:
        print(str(x) + ', ' + str(chr(x)))
    '''

def deg2rad(_in):
    _out = [(j/180.0*3.14) for j in _in]
    return _out

if __name__ == '__main__':
    main()

