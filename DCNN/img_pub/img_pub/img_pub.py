#!/usr/bin/env python3

import rclpy
# Imports the ROS message String from the ROS package std_msgs
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from custom_msg.srv import DetectionSrv
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge



def cam_sub_callback(msg):
    global sensor_img
    sensor_img = msg

# define callback function
def pub_callback():
    global pub, sensor_img
    pub.publish(sensor_img)
    # print('published sensor_img')

def det_sub_callback(msgs):
    global detection, score
    
    detection = String()
    score = 0.0

    i = 0
    while i < len(msgs.detections):
        j = 0
        while j < len(msgs.detections[i].results):
            if score < msgs.detections[i].results[j].score:
                detection = msgs.detections[i].results[j].id
                score = msgs.detections[i].results[j].score
            j = j +1

        i = i +1

    print('detected Img: ' + str(detection) + ' (score = ' + str(score) + ')')
           
def call_detection_callback(request, response):
    global detection

    pub_callback()

    response.detection = detection
    
    print('Incoming request.')
    print(['Detection: ', detection])

    detection = ''

    return response

def main():
    global pub, sensor_img, detection, score

    sensor_img = Image()
    detection = ''
    score = 0.0

    # initialize node
    rclpy.init()
    node_handle = rclpy.create_node('detector_conector')
    # initialize subscriber
    node_handle.create_subscription(Image, '/camera/image_raw', cam_sub_callback , 10)
    # initialize publisher AND declare timer with a callback every 3 seconds
    pub = node_handle.create_publisher(Image, '/detector_node/images', 1)
    node_handle.create_timer(2, pub_callback)
    # initialize subscribtion for detection
    node_handle.create_subscription(Detection2DArray, '/detector_node/detections', det_sub_callback , 10)
    # initialize Service Server
    srv = node_handle.create_service(DetectionSrv, 'call_detection', call_detection_callback)
 
    try:
        rclpy.spin(node_handle)
    except KeyboardInterrupt:
        pass
    
    node_handle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()