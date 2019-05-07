# import the necessary packages
import numpy as np
import argparse
import cv2
import cv2.cv as cv 
import time
import baxter_interface 
import roslib
import rospy
import cv_bridge
from sensor_msgs.msg import Image

rospy.init_node('camarita2',anonymous=True)

cap = baxter_interface.CameraController('left_hand_camera') # Set Capture Device, in case of a USB Webcam try 1, or give -1 to get a list of available devices
cap.open()

cap.resolution = cap.MODES[0]
#Set Width and Height 
#cap.set(3,1280)
#cap.set(4,720)
foto = None

# The above step is to set the Resolution of the Video. The default is 640x480.
# This examp8le works with a Resolution of 640x480.
def callback(msg):
	global foto
	foto = cv_bridge.CvBridge().imgmsg_to_cv2(msg)

rospy.Subscriber('/cameras/left_hand_camera/image', Image, callback)

while not rospy.is_shutdown():
	# Capture frame-by-frame
	while np.all(foto) == None:
		print "alo"
		continue 

	#ret, frame = cap.read()
 
	frame= foto
	# load the image, clone it for output, and then convert it to grayscale
			
	#cv.CvtColor(cv.fromarray(foto),gray1, cv.CV_BGR2GRAY)
	#kernel = np.ones((6,6),np.uint8)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	dimensiones =gray.shape
        canny= cv2.Canny(gray,50,150)
        cv2.imshow("Canny",canny)
        # apply GuassianBlur to reduce noise. medianBlur is also added for smoothening, reducing noise.
  
	#gray = cv2.morphologyEx(gray,cv2.MORPH_CLOSE,kernel)
	# Adaptive Guassian Threshold is to detect sharp edges in the Image. For more information Google it.
	
	
	#gray = cv2.erode(gray,kernel,iterations = 1)
	# gray = erosion
	#Se aplica la transformacion: Closing
        
	#gray = cv2.dilate(gray,kernel,iterations = 1)
	# gray = dilation

	# get the size of the final image
	# img_size = gray.shape
	# print img_size	
	
	# detect circles in the image
	
	#print circles
        contornos,_ = cv2.findContours(canny.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print("He encontrado {} objetos".format(len(contornos)))
 
        cv2.drawContours(frame,contornos,0,(0,0,255), 2)
        cv2.imshow("contornos", frame)
 
        cv2.waitKey(0)
        
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
