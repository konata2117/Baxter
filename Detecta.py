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
        RGB=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        cv2.imshow("RGB",RGB)
        # apply GuassianBlur to reduce noise. medianBlur is also added for smoothening, reducing noise.
        kernel = np.ones((3,3),np.uint8)
	gray = cv2.GaussianBlur(gray,(5,5),0);
	gray = cv2.medianBlur(gray,5)
        gray = cv2.erode(gray,kernel,iterations = 1)
	#gray = cv2.morphologyEx(gray,cv2.MORPH_CLOSE,kernel)
	# Adaptive Guassian Threshold is to detect sharp edges in the Image. For more information Google it.
	gray = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,3.5)
	
	
	#gray = cv2.erode(gray,kernel,iterations = 1)
	# gray = erosion
	#Se aplica la transformacion: Closing
        
	#gray = cv2.dilate(gray,kernel,iterations = 1)
	# gray = dilation

	# get the size of the final image
	# img_size = gray.shape
	# print img_size	
	
	# detect circles in the image
	circles = cv2.HoughCircles(gray, cv.CV_HOUGH_GRADIENT, 1, 200, param1=40, param2=45, minRadius=20, maxRadius=140)
	#print circles
	
	# ensure at least some circles were found
	if circles is not None:
		# convert the (x, y) coordinates and radius of the circles to integers
		circles = np.round(circles[0, :]).astype("int")
		
		# loop over the (x, y) coordinates and radius of the circles
		for (x, y, r) in circles:
			# draw the circle in the output image, then draw a rectangle in the image
			# corresponding to the center of the circle
			cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
			cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
			#time.sleep(0.5)
			#print "Column Number: "
			#print x
			#print "Row Number: "
			#print y
			print "Radius is: "
			print r

	# Display the resulting frame
        cv2.imshow('gray',gray)
    	cv2.imshow('frame',frame)
 	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
