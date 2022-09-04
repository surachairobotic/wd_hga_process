import socket, cv2, pickle, struct, imutils, time
import pyrealsense2 as rs
import numpy as np
import cv2

devide = 2

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

  pipeline = rs.pipeline()
  config = rs.config()

  pipeline_wrapper = rs.pipeline_wrapper(pipeline)
  pipeline_profile = config.resolve(pipeline_wrapper)
  device = pipeline_profile.get_device()
  device_product_line = str(device.get_info(rs.camera_info.product_line))

  found_rgb = False
  for s in device.sensors:
    print(s.get_info(rs.camera_info.name))
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
      found_rgb = True
      break
  if not found_rgb:
    print('RGB Camera not found.')
    exit()

  config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
  pipeline.start(config)

  try:
    while True:
      frames = pipeline.wait_for_frames()
      color_frame = frames.get_color_frame()
      #depth_frame = frames.get_depth_frame()
      if not color_frame:
        continue      
      color_image = np.asanyarray(color_frame.get_data())
      cv2.imshow('RealSense', color_image)
      key = cv2.waitKey(1) & 0xFF
      if key == ord('q') or key == ord('p'):
        break
  finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()

  print('OK')

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

  pipeline.start(config)
  bExit = False
  while not bExit:
    client_socket, addr = server_socket.accept()
    print('GOT CONNECTION FROM: ', addr)
    #if (cv2.waitKey(1) & 0xFF) == ord('w'):
    #    break
    try:
      while client_socket:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
          continue      
        color_image = np.asanyarray(color_frame.get_data())

        a = pickle.dumps(color_image)
        message = struct.pack("Q", len(a)) + a
        try:
          client_socket.sendall(message)
        except:
          client_socket.close()
          break

        cv2.imshow('Server VDO', color_image)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('w'):
          vid.release()
          client_socket.close()
        elif key == ord('q'):
          vid.release()
          client_socket.close()
          bExit = True
    except:
      # Stop streaming
      pipeline.stop()
      cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
