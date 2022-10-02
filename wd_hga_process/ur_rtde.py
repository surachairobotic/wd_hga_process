import argparse
import logging
import sys
sys.path.append('..')
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import time, threading

class UR_INFORMATION():
    def __init__(self):
        self.j = []
        self.v = []
    
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
        self.args = parser.parse_args()

        if self.args.verbose:
            logging.basicConfig(level=logging.INFO)

        conf = rtde_config.ConfigFile(self.args.config)
        output_names, output_types = conf.get_recipe('out')

        self.con = rtde.RTDE(self.args.host, self.args.port)
        self.con.connect()

        # settings
        self.con.get_controller_version()
        self.con.send_output_setup(output_names, output_types, frequency=self.args.frequency)
        self.con.send_start()
        
        self.thread = threading.Thread(target=self.threadGetInfo)
        self.thread.start()

    def __del__(self):
        self.thread.join()
        self.con.send_pause()
        self.con.disconnect()
    
    def threadGetInfo(self):
        while True:
            if self.args.samples > 0 and i >= self.args.samples:
                keep_running = False
            try:
                if self.args.buffered:
                    state = self.con.receive_buffered(self.args.binary)
                else:
                    state = self.con.receive(self.args.binary)
                if state is not None:
                    self.j = state.actual_q
                    self.v = state.actual_qd
                    X,Y,Z,RX,RY,RZ = state.actual_TCP_pose
                    date_and_time = state.timestamp
                    #print("TCP: pos ["+str(X)+", "+str(Y)+", "+str(Z)+"] m, rot ["+str(RX)+", "+str(RY)+", "+str(RZ)+"] rad")
                    #print(j)
                    #print(self.v)
                    time.sleep(0.1)

            except KeyboardInterrupt:
                break
            except rtde.RTDEException:
                break

