
import argparse
import logging
import sys
sys.path.append('..')
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import time, keyboard
#from universal_robot_kinematics import *
from ur5_kinematics import *
#from rtde_control import RTDEControlInterface as RTDEControl

#rtde_c = RTDEControl("192.168.12.100")

kin = Kinematic()

# parameters
parser = argparse.ArgumentParser()
parser.add_argument('--host', default='192.168.12.100',help='name of host to connect to (localhost)')
parser.add_argument('--port', type=int, default=30004, help='port number (30004)')
parser.add_argument('--samples', type=int, default=0,help='number of samples to record')
parser.add_argument('--frequency', type=int, default=125, help='the sampling frequency in Herz')
parser.add_argument('--config', default='record_configuration.xml', help='data configuration file to use (record_configuration.xml)')
parser.add_argument("--verbose", help="increase output verbosity", action="store_true")
parser.add_argument("--buffered", help="Use buffered receive which doesn't skip data", action="store_true")
parser.add_argument("--binary", help="save the data in binary format", action="store_true")
args = parser.parse_args()

if args.verbose:
    logging.basicConfig(level=logging.INFO)

conf = rtde_config.ConfigFile(args.config)
output_names, output_types = conf.get_recipe('out')

con = rtde.RTDE(args.host, args.port)
con.connect()

# settings
con.get_controller_version()
con.send_output_setup(output_names, output_types, frequency=args.frequency)
con.send_start()

# initialize variables
X = 0
Y = 0
Z = 0
RX = 0
RY = 0
RZ = 0
# main loop
i = 1
while True:
    if args.samples > 0 and i >= args.samples:
        keep_running = False
    try:
        if args.buffered:
            state = con.receive_buffered(args.binary)
        else:
            state = con.receive(args.binary)
        if state is not None:
            j = state.actual_q
            jj = state.actual_qd
            X,Y,Z,RX,RY,RZ = state.actual_TCP_pose
            date_and_time = state.timestamp
            i += 1
            print("Joint : {}".format(j))
            print("TCP: pos ["+str(X)+", "+str(Y)+", "+str(Z)+"] m, rot ["+str(RX)+", "+str(RY)+", "+str(RZ)+"] rad")
            #joint = rtde_c.getInverseKinematics([X,Y,Z], [RX,RY,RZ], 1.0, 1.0)
            #joint = kin.invKine(state.actual_TCP_pose, j)
            #print(joint)
            #print(jj)
            time.sleep(0.1)

    except KeyboardInterrupt:
        break
    except rtde.RTDEException:
        break

con.send_pause()
con.disconnect()
