import socket, cv2, pickle, struct, imutils, time
import pyrealsense2 as rs
import numpy as np
import cv2, select

devide = 1

def returnCameraIndexes():
  # checks the first 10 indexes.
  index = 0
  arr = []
  while index < 10:
    print('index = ', index)
    cap = cv2.VideoCapture(index)
    if cap.read()[0]:
      #print(cap)
      arr.append(index)
      cap.release()
    index += 1
  return arr

def main():
    global devide

    indx = returnCameraIndexes()    
    indx_selected = 0
    while True:
        vid = cv2.VideoCapture(indx[indx_selected])
        #vid.set(cv2.CAP_PROP_BUFFERSIZE, 10)
        vid.set(cv2.CAP_PROP_FRAME_WIDTH, 848)
        vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        f = vid.get(cv2.CAP_PROP_FPS)
        print('FPS : ', f)
        width = int(vid.get(3)/devide)
        height = int(vid.get(4)/devide)
        print(width, " : ", height)
        #exit()
        while( vid.isOpened() ):
            t = time.time()
            #print('Press q to exit and p for continue.')
            ret, frame = vid.read()
            #print('t1 : ', time.time()-t)
            t2 = time.time()
            if ret:
                #cv2.imshow('Server VDO1', frame)
                frame = cv2.resize(frame, (width, height))
                cv2.imshow('Server VDO2', frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == ord('p'):
                    cv2.destroyAllWindows()
                    vid.release()
                    break
                if key == ord('n'):
                    indx_selected = (indx_selected+1) % len(indx)
                    break
            #print('t2 : ', time.time()-t2)
            fps = 1.0/(time.time()-t)
            if fps < f-1:
                print('[{}]-FPS : {}'.format(indx[indx_selected], fps))
        if key == ord('q'):
            exit()
        elif key == ord('p'):
            break

    # Socket Create
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host_name = socket.gethostname()
    host_ip = socket.gethostbyname(host_name)
    print('HOST IP: ', host_ip)
    host_ip = "0.0.0.0"
    print('Enter port : ')
    port = int(input())
    print('Local IP : {}'.format(socket.gethostbyname_ex(socket.gethostname())[-1]))
    
    socket_address = (host_ip, port)

    #Socket Bind
    server_socket.bind(socket_address)

    # Socket Listen
    server_socket.listen(5)
    print('LISTENING AT: ', socket_address)

    read_list = [server_socket]

    bExit = False
    while not bExit:
        readable, writable, errored = select.select(read_list, [], [])
        print("len(readable) : {}".format(len(readable)))
        for s in readable:
            print("s : {}".format(type(s)))
            if s is server_socket:
                client_socket, address = server_socket.accept()
                #read_list.append(client_socket)
                print("Connection from {}".format(address))

                vid = cv2.VideoCapture(indx[indx_selected])
                vid.set(cv2.CAP_PROP_FRAME_WIDTH, 848)
                vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                width = int(vid.get(3)/devide)
                height = int(vid.get(4)/devide)
                  
                while( vid.isOpened() ):
                    print('Press q to exit and w for continue.')
                    ret, frame = vid.read()
                    if ret:
                        frame = cv2.resize(frame, (width, height))      
                        #frame = imutils.resize(frame, width=320)

                        a = pickle.dumps(frame)
                        message = struct.pack("Q", len(a)) + a
                        try:
                            client_socket.sendall(message)
                        except:
                            break
                          
                        cv2.imshow('Server VDO', frame)
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord('w'):
                            vid.release()
                            client_socket.close()
                            read_list.remove(s)
                        elif key == ord('q'):
                            vid.release()
                            client_socket.close()
                            read_list.remove(s)
                            bExit = True


if __name__ == '__main__':
    main()
