import socket, cv2, pickle, struct, imutils, time
import pyrealsense2 as rs
import numpy as np
import cv2

devide = 1

def returnCameraIndexes():
  # checks the first 10 indexes.
  index = 0
  arr = []
  i = 10
  while i > 0:
    print('i1 = ', i)
    cap = cv2.VideoCapture(index)
    if cap.read()[0]:
      arr.append(index)
      cap.release()
    index += 1
    i -= 1
  return arr

def main():
  global devide
  indx = returnCameraIndexes()
  print(indx)
  print(indx[-1])

  vid = cv2.VideoCapture(indx[-1])
  #vid.set(cv2.CAP_PROP_BUFFERSIZE, 10)
  vid.set(cv2.CAP_PROP_FRAME_WIDTH, 848)
  vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
  f = vid.get(cv2.CAP_PROP_FPS)
  print('FPS : ', f)
  print(vid.get(3), " : ", vid.get(4))
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
    #print('t2 : ', time.time()-t2)
    fps = 1.0/(time.time()-t)
    if fps < f-1:
      print('FPS : ' + str(fps))
  if key == ord('q'):
    exit()

  # Socket Create
  server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  host_name = socket.gethostname()
  host_ip = socket.gethostbyname(host_name)
  print('HOST IP: ', host_ip)
  host_ip = "0.0.0.0"
  print('Enter port : ')
  port = int(input())
  socket_address = (host_ip, port)

  #Socket Bind
  server_socket.bind(socket_address)

  # Socket Listen
  server_socket.listen(5)
  print('LISTENING AT: ', socket_address)

  bExit = False
  while not bExit:
    print('server_socket.accept()')
    client_socket, addr = server_socket.accept()
    print('GOT CONNECTION FROM: ', addr)
    #if (cv2.waitKey(1) & 0xFF) == ord('w'):
    #    break
    if client_socket:
      vid = cv2.VideoCapture(indx[-1])
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
          elif key == ord('q'):
            vid.release()
            client_socket.close()
            bExit = True

if __name__ == '__main__':
    main()
