#!/usr/bin/env python
import rospy
import cv2
import random as rng
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import find_peaks
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
from std_msgs.msg import Float32MultiArray

bridge = CvBridge()

####### PROCESSING IMAGE FUNCTIONS ###############################

def warp_image(frame):
    # Transformation coordinates
    frame_copy = frame.copy()
    topLeft = (55,400)
    bottomLeft = (0,435)
    topRight = (515,400)
    bottomRight = (565,435)
    cv2.circle(frame_copy,topLeft,5,(255,0,0),5)
    cv2.circle(frame_copy,bottomRight,5,(255,0,0),5)
    cv2.circle(frame_copy,topRight,5,(255,0,0),5)
    cv2.circle(frame_copy,bottomLeft,5,(255,0,0),5)

    # Apply Transformation
    pts1 = np.float32([topLeft,bottomLeft,topRight,bottomRight])
    pts2 = np.float32([[0,0], [0,480], [640,0], [640,480]])
    # Eye's view
    matrix = cv2.getPerspectiveTransform(pts1,pts2)
    transformed_frame = cv2.warpPerspective(frame, matrix, (640,480))
    return transformed_frame,frame_copy

def thresh(image):
    # Llevar a escala de grises
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    # Filtro
    gray = cv2.GaussianBlur(gray, (5,5), 0)
    # Thresholding
    _, thresh_image = cv2.threshold(gray,100,255,cv2.THRESH_BINARY)
    return thresh_image

def canny(image):
    canny_output = cv2.Canny(image,100,255,apertureSize=3)
    return canny_output

def div(image):
    middle_point = int(image.shape[1]/2)
    img_left = image[:,:middle_point]
    img_right = image[:,middle_point:]
    return img_left,img_right

def histog(image):
    hist = np.sum(image[image.shape[0]//2:,:],axis=0)
    return hist

def drawing(center_left_lane, left_img, center_right_lane, right_img, edges_img,y):
    # Drawing windows
    size = (41, 21)
    center = (center_left_lane,y)
    cv2.rectangle(left_img, (center[0]-size[0]//2, center[1]-size[1]//2), (center[0]+size[0]//2, center[1]+size[1]//2), (255, 255, 255), 2)
    center = (center_right_lane,y)
    cv2.rectangle(right_img, (center[0]-size[0]//2, center[1]-size[1]//2), (center[0]+size[0]//2, center[1]+size[1]//2), (255, 255, 255), 2)
    # Middle points between centers of windows
    middle_img = int(edges_img.shape[0]/2)
    middle_point = center_left_lane + int(((middle_img +center_right_lane)-center_left_lane)/2)
    cv2.circle(edges_img, (middle_point,y),5,(255,255,255),2)
    return middle_point

############### INTERMEDIATE FUNCTIONS #########

def eucl_distance(P1,P2):
    x1 = P1[0]
    y1 = P1[1]
    x2 = P2[0]
    y2 = P2[1]
    term1 = x2-x1
    term2 = y2-y1 
    return np.sqrt(term1**2+term2**2)

def pub_velocities(v,w):
    pub = rospy.Publisher('velocities', Float32MultiArray, queue_size=10)
    velocities = [v, w]
    msg = Float32MultiArray()
    msg.data = velocities
    print('Published')
    pub.publish(msg)
    

    
############ MSGS_CALLBACK ##################

def image_callback(ros_image):
    global bridge
    # imagen ros a compatible con opencv
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image,"bgr8")
    except CvBridgeError as e:
        print(e)
    frame = cv2.resize(cv_image,(640,480))
    
    # Aereal Perspective
    warped_img, frame_copy = warp_image(frame)
    # Thresh image
    thresh_img = thresh(warped_img)
    # Edges detection
    edges_img = canny(thresh_img)
    # Divide edges image
    left_img,right_img = div(edges_img)
    # Histogram determination
    left_hist = histog(left_img)
    right_hist = histog(right_img)
    # Middle points between peaks
    left_peaks, _ = find_peaks(left_hist, height=2000)
    right_peaks, _ = find_peaks(right_hist, height=2000)
    pub_velocities(0.05,0.00)
    print("peaks")
    if right_peaks.size > 1:
        center_right_lane = right_peaks[-1] - int((right_peaks[-1]-right_peaks[0])/2)
    elif right_peaks.size == 1:
        center_right_lane = right_peaks[-1]
        pub_velocities(0.04,0.01)
    else:
        # Controles
        # Giro derecha +++
        pub_velocities(0.02,-0.05)
    if left_peaks.size > 1:
        center_left_lane = left_peaks[0] + int((left_peaks[-1]-left_peaks[0])/2)
    elif left_peaks.size == 1:
        center_left_lane = left_peaks[0]
        pub_velocities(0.04,-0.01)
    else:        
        # Controles
        # Giro izquierda +++
        pub_velocities(0.02,0.05)
    print("draw")
    # Drawing Window
    y = 420
    if right_peaks.size != 0 and left_peaks.size != 0:
        middle_point = drawing(center_left_lane, left_img, center_right_lane, right_img, edges_img,y)
    
        # Calculating distances
        #D = 50
        #P1 = [center_left_lane,y]
        #P = [middle_point,y]
        #P2 = [center_right_lane,y]
        #d1 = eucl_distance(P1,P)
        #d2 = eucl_distance(P,P2)
        """
        # Applying smooth controls
        if D-d1 > 0:
            # Control
            pub_velocities(0.04,-0.01)
        for i in range(30):
            pass
        pub_velocities(0.05,0.00)
        if D-d2 > 0:
            # Control
            pub_velocities(0.04,0.01)
        for i in range(30):
            pass
        pub_velocities(0.05,0.00)"""
    
    #pub_velocities(0.1,0.00)
    


    
    #Visualization
    cv2.imshow("Warped Image",warped_img)
    cv2.imshow("Image",frame_copy)
    cv2.imshow("Edges Image",edges_img)
    cv2.waitKey(3)
    
########### MAIN ###############################

def main(args):
    rospy.init_node('image_converter', anonymous=True)
    # para turtlebot3 waffle_pi
    image_topic = "/camera/rgb/image_raw"
    image_sub = rospy.Subscriber(image_topic,Image,image_callback)  
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    #cv2.destroyAllWindows()

if __name__== '__main__':
    main(sys.argv)