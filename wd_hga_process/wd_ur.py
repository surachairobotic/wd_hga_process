import socket, time, keyboard

from urx import Robot
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
import math3d as m3d

def main():
    robot = Robot("192.168.12.100")
    grip = Robotiq_Two_Finger_Gripper(robot=robot, socket_host="192.168.12.100")
    
    #print(trans)
    trans = robot.get_pose()
    print(trans.pos)
    
    #grip.gripper_action(100)
    #time.sleep(2.0)
    #grip.close_gripper()
    #exit()
    #print(trans.orient)
    
    #trans.pos.x -= 0.01
    
    #robot.set_pose(trans, acc=0.5, vel=0.2)

    #print(trans.pos)
    #print(trans.orient)
    while True:
        trans = robot.get_pose()
        
        bIn = False
        if keyboard.is_pressed('q'):
            break
        elif keyboard.is_pressed('w'):
            bIn = True
            trans.pos.x += 0.01
        elif keyboard.is_pressed('s'):
            bIn = True
            trans.pos.x -= 0.01
        elif keyboard.is_pressed('a'):
            bIn = True
            trans.pos.y += 0.01
        elif keyboard.is_pressed('d'):
            bIn = True
            trans.pos.y -= 0.01
        elif keyboard.is_pressed('+'):
            bIn = True
            grip.open_gripper()
        elif keyboard.is_pressed('-'):
            bIn = True
            grip.close_gripper()

        if bIn:
            print('move')
            robot.set_pose(trans, acc=0.5, vel=0.2)

        print(trans.pos)
        time.sleep(0.1)

    robot.close()

def old_main(args=None):
    HOST = "192.168.12.100"
    PORT = 30002
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    
    d = [90,-90,140,-237,-90,180]
    j = deg2rad(d)
    print(d)
    print(j)

    #cmd = "set_digital_out(0,False)" + "\n"
    #cmd = "movej([1.56,-1.57,2.45,-4.13,-1.57,3.14],a=1.4,v=1.05,t=0,r=0)" + "\n"
    cmd = "get_actual_joint_positions()" + "\n"    
    
    s.send(cmd.encode('utf-8'))
    data = s.recv(1024)
    
    s.close()
    
    print('Recv : ', str(data))
    print('-----')
    for x in data:
        print(str(x) + ', ' + str(chr(x)))
    

def deg2rad(_in):
    _out = [(j/180.0*3.14) for j in _in]
    return _out

if __name__ == '__main__':
    main()
