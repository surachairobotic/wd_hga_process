from os import listdir
from os.path import isfile, join

import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile
import cv2

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image

import IPython.display as display

sys.path.append("..")
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

nm = 'overpass_m33-37'
path = '/home/probook/catkin_ws/src/datmo/data/'
mypath = path+nm

# What model to download.
# Models can bee found here: https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md
MODEL_NAME = 'ssd_inception_v2_coco_2017_11_17'
#MODEL_NAME = 'ssd_inception_v2_coco_2018_01_28'

# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = '/home/probook/catkin_ws/src/datmo/models/' + MODEL_NAME + '/frozen_inference_graph.pb'

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = '/home/probook/deep_ws/models/research/object_detection/data/mscoco_label_map.pbtxt'

# Number of classes to detect
NUM_CLASSES = 90

def main():
    global mypath, NUM_CLASSES, PATH_TO_LABELS, PATH_TO_CKPT, MODEL_NAME, nm
    files = [f for f in listdir(mypath) if isfile(join(mypath, f)) and f.find('.jpg') != -1]
    print(type(files))
    print(files)
    files.sort()
    print(type(files))
    print(files)
    length = len(files)
    print(length)
    print(files)
    mypath = mypath+'/'
    #print(mypath+files[0])
    img = cv2.imread(mypath+files[0])
    #print(type(img))
    #print(img.shape)
    height = img.shape[0]
    width = img.shape[1]
    fps = 30
    out = cv2.VideoWriter(path+nm+'.avi',cv2.VideoWriter_fourcc('M','J','P','G'), fps, (width, height))

    cnt=0

    print('PATH_TO_LABELS : ', PATH_TO_LABELS)
    print('PATH_TO_CKPT : ', PATH_TO_CKPT)

    # Load a (frozen) Tensorflow model into memory.
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.compat.v1.GraphDef()
        with tf.io.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')


    # Loading label map
    # Label maps map indices to category names, so that when our convolution network predicts `5`, we know that this corresponds to `airplane`.  Here we use internal utility functions, but anything that returns a dictionary mapping integers to appropriate string labels would be fine
    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    categories = label_map_util.convert_label_map_to_categories(
        label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
    category_index = label_map_util.create_category_index(categories)

    with detection_graph.as_default():
        with tf.compat.v1.Session(graph=detection_graph) as sess:
            #while True:
            for f in files:
                if f.find('.jpg') != -1:
                    #print(f)
                    # Read frame from camera
                    image_np = cv2.imread(mypath+f)
                    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                    image_np_expanded = np.expand_dims(image_np, axis=0)
                    # Extract image tensor
                    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
                    # Extract detection boxes
                    boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
                    # Extract detection scores
                    scores = detection_graph.get_tensor_by_name('detection_scores:0')
                    # Extract detection classes
                    classes = detection_graph.get_tensor_by_name('detection_classes:0')
                    # Extract number of detectionsd
                    num_detections = detection_graph.get_tensor_by_name(
                        'num_detections:0')
                    # Actual detection.
                    (boxes, scores, classes, num_detections) = sess.run(
                        [boxes, scores, classes, num_detections],
                        feed_dict={image_tensor: image_np_expanded})

                    print('type: %s, %s' % (type(boxes), boxes))
                    # Visualization of the results of a detection.
                    vis_util.visualize_boxes_and_labels_on_image_array(
                        image_np,
                        np.squeeze(boxes),
                        np.squeeze(classes).astype(np.int32),
                        np.squeeze(scores),
                        category_index,
                        use_normalized_coordinates=True,
                        line_thickness=1)

                    # Display output
                    #cv2.imshow('object detection', cv2.resize(image_np, (800, 600)))

                    #out.write(image_np)            
                    print( '%d of %d' % (cnt, length) )
                    cnt=cnt+1

                    if cv2.waitKey(25) & 0xFF == ord('q'):
                        cv2.destroyAllWindows()
                        break


if __name__ == "__main__":
    #rospy.init_node('detect', anonymous=True)
    main()

