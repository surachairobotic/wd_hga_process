# lets make the client code
import socket, cv2, pickle, struct, time
import numpy as np

def colorDetection(img):
    # convert to hsv colorspace
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # lower bound and upper bound for Green color
    lower_bound = np.array([50,50,50])	 
    upper_bound = np.array([150,255,255])

    # find the colors within the boundaries
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    
    res = cv2.bitwise_and(img, img, mask=mask)

    return res

def main():
    # create socket
    client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    host_ip = '192.168.8.170' # paste your server ip address here
    port = 9998
    client_socket.connect((host_ip,port)) # a tuple
    data = b""
    payload_size = struct.calcsize("Q")
    while True:
        t = time.time()
        while len(data) < payload_size:
            packet = client_socket.recv(4*1024) # 4K
            if not packet: break
            data+=packet
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack("Q",packed_msg_size)[0]
	    
        while len(data) < msg_size:
            data += client_socket.recv(4*1024)
        frame_data = data[:msg_size]
        data  = data[msg_size:]
        frame = pickle.loads(frame_data)
        #frame2 = colorDetection(frame)
        cv2.imshow("RECEIVING VIDEO", frame)
        print('FPS : ' + str(1.0/(time.time()-t)))
        key = cv2.waitKey(1) & 0xFF
        if key  == ord('q'):
            break
    client_socket.close()
	
if __name__ == '__main__':
    main()
