import rclpy

# lets make the client code
import socket, cv2, pickle, struct, time
import numpy as np
import threading
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class IMAGE():
    def __init__(self, node):
        self.node = node
        self.threadImg = None

        self.pub_img = self.node.create_publisher(Image, '/camera/color/image_raw', 10)
        
        # create socket
        self.client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        host_ip = '192.168.12.253' # paste your server ip address here
        port = 2222
        self.client_socket.connect((host_ip,port)) # a tuple
        self.data = b""
        self.payload_size = struct.calcsize("Q")

        self.bridge = CvBridge()

        self.threadImg = threading.Thread(target=self.threadRecv)
        self.threadImg.start()
    
    def __del__(self):
        if self.threadImg:
            self.threadImg.join()
        self.client_socket.close()
    
    def threadRecv(self):
        while True:
            t = time.time()
            while len(self.data) < self.payload_size:
                packet = self.client_socket.recv(4*1024) # 4K
                if not packet:
                    break
                self.data+=packet
            packed_msg_size = self.data[:self.payload_size]
            self.data = self.data[self.payload_size:]
            msg_size = struct.unpack("Q",packed_msg_size)[0]
	        
            while len(self.data) < msg_size:
                self.data += self.client_socket.recv(4*1024)
            frame_data = self.data[:msg_size]
            self.data  = self.data[msg_size:]
            frame = pickle.loads(frame_data)
            cv2.imshow("RECEIVING VIDEO", frame)
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
            try:
                self.pub_img.publish(image_message)
            except CvBridgeError as e:
                print(e)

            print('FPS : ' + str(1.0/(time.time()-t)))
            cv2.waitKey(1)    

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('vdo_over_ip_client_rospub')
    
    img = IMAGE(node)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()
	
if __name__ == '__main__':
    main()
