#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
import math
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import tf

rospy.init_node('camarita2',anonymous=True)
cap = baxter_interface.CameraController('right_hand_camera') 
cap.open()
cap.resolution = cap.MODES[0]
foto = None
def QtoE(): #Quaternion to Euler. Converts Quaternion angles(x, y, z, w) into Euler angles (x, y ,z) and prints them
		euler = tf.transformations.euler_from_quaternion(limb_interface.endpoint_pose()['orientation'])
		print ("Arm positions and Quaternion angles")
		print (limb_interface.endpoint_pose())
		print ("Arm Euler angles: ", euler)
# Pose inicial 47 x27 cm
xx = 0.47
yy = -0.52
zz = 0.10
roll = math.pi	#Rotacion x
pitch = 0.0	#Rotacion y	
yaw = 0.0		#Rotacion z

pose_i = [xx, yy, zz,roll,pitch, yaw]
pose = [xx,yy,zz,roll, pitch, yaw]
gripper = baxter_interface.Gripper("right")
limb_interface = baxter_interface.Limb('right')
# calibrate the gripper
gripper.calibrate()

# Parametros de camara
cam_calibracion = 0.025            # 0.0025 pixeles por metro a 1 metro de distancia. Factor de correccion
cam_x_offset    = -0.045       # Correccion de camara por los gripper, 0.04 / -0.015
cam_y_offset    = -0.160  
resolution      = 1
width           = 960               # 1280 640  960
height          = 600               # 800  400  600
#The above step is to set the Resolution of the Video. The default is 640x480.
# This examp8le works with a Resolution of 640x480.

def callback(msg):
	global foto
	foto = cv_bridge.CvBridge().imgmsg_to_cv2(msg)

def send_image(image):	#Shows an image on Baxter's screen
	img = cv2.imread(image)	#Reads an image
	msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8") #Makes the opencv-ros bridge, converts an image to msg
	pub.publish(msg) #Shows the image on Baxter's head screen
	rospy.sleep(0.1)

def mensaje_matriz_a_pose(T, frame):
	t = PoseStamped()
	t.header.frame_id = frame
	t.header.stamp = rospy.Time.now()
	translacion = tf.transformations.translation_from_matrix(T)
	orientacion = tf.transformations.quaternion_from_matrix(T)
	t.pose.position.x = translacion[0]
	t.pose.position.y = translacion[1]
	t.pose.position.z = translacion[2]
	t.pose.orientation.x = orientacion[0]
	t.pose.orientation.y = orientacion[1]
	t.pose.orientation.z = orientacion[2]
	t.pose.orientation.w = orientacion[3]        
	return t

def pixel_to_baxter(px, dist):
	

	print "px[0]", px
	print "pose_i: ",pose_i
	x = ((px[0] - (height / 2)) * cam_calibracion * dist)\
	    + pose_i[0] + cam_x_offset

	y = ((px[1] - (width / 2)) * cam_calibracion * dist)\
	     + pose_i[1] + cam_y_offset
    
	print x , y
	return (x, y)

def mover_baxter(source_frame, trans, rot):
	nombre_servicio = '/ExternalTools/'+ 'right' +'/PositionKinematicsNode/IKService'
	servicio_ik = rospy.ServiceProxy(nombre_servicio,SolvePositionIK)
	frame = source_frame   

	# Promedio de velocidad del brazo
	limb_interface.set_joint_position_speed(0.5)

	matrix = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans),
		tf.transformations.euler_matrix(rot[0],rot[1],rot[2])
		)
		
	rospy.wait_for_service(nombre_servicio,10)
	ik_mensaje = SolvePositionIKRequest()
	ik_mensaje.pose_stamp.append(mensaje_matriz_a_pose(matrix, frame))

	try:
		respuesta = servicio_ik(ik_mensaje)
	except:
		print "Movimiento no ejecutado"
	print respuesta.isValid[0]

	if respuesta.isValid[0] == True:
		movimiento =  dict(zip(respuesta.joints[0].name, respuesta.joints[0].position))
		limb_interface.move_to_joint_positions(movimiento)
	else:
		print "Movimiento no ejecutado"
		print respuesta.joints[0].position
		print respuesta.joints[0].name

        # 3ms wait
#cv.WaitKey(3)


def transformacion(frame):
	hh,ww=frame.shape[:2]
	roi=frame[300:hh, 0:ww]
	gray= cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	#gray= cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
	blur= cv2.GaussianBlur(gray,(5,5),0)
	#canny= cv2.Canny(blur,50,200)
	canny= cv2.Canny(blur,20,100)

	#Morphologic, para completar bordes
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
	dilated = cv2.dilate(canny,kernel)

	cv2.imwrite('cam_dilated.jpg',dilated)
	#circles =cv2.HoughCircles(dilated,cv.CV_HOUGH_GRADIENT,1, 200, param1=50, param2=65,minRadius=20, maxRadius=160)
	(contornos,_) = cv2.findContours(dilated,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	return contornos
	'''
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #tranformacion de la imagen a escla de grises
	dimensiones =gray.shape
	kernel = np.ones((3,3),np.uint8)
	gray = cv2.GaussianBlur(gray,(5,5),0)#para eliminar el ruido de la imagen
	gray = cv2.medianBlur(gray,5) #reduce el ruido de sal
	gray = cv2.erode(gray,kernel,iterations = 1)#erosiona la imagen
	gray = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,3.5)
	cv2.imshow("GR",gray)
	#detectar circulos en la imagen
	circles =cv2.HoughCircles(gray,cv.CV_HOUGH_GRADIENT,1, 200, param1=50, param2=65,minRadius=20, maxRadius=160)
	#circles = cv2.HoughCircles(gray, cv.CV_HOUGH_GRADIENT, 1, 200, param1=40, param2=45, minRadius=20, maxRadius=140)
	return circles 
	''' 
def dibujarcirculo(contornos,frame):
	#print "circles" , contornos
	if contornos is not None:

		# convert the (x, y) coordinates and radius of the circles to integers
		#contornos = np.round(contornos[0, :]).astype("int")
		
		# loop over the (x, y) coordinayates and radius of the circles
		for c in contornos:	
			x,y,w,h=cv2.boundingRect(c)
			cv2.circle(frame, (x, y), w, (0, 255, 0), 4) #dibujar circulo periferico
			cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1) #dibujar rectangulo en el centro
		cv2.imshow('circulos',frame)

#right_camera_callback(data)
rospy.Subscriber('/cameras/right_hand_camera/image', Image, callback)

#while not rospy.is_shutdown():
QtoE()
mover_baxter('base',[xx,yy,zz],[math.pi,0,0])
circle	=[]
while not rospy.is_shutdown():	# Capture frame-by-frame
	while np.all(foto) == None:
	#	print "Ahorita no joven"
		continue
	frame=foto
	circles=transformacion(frame)
	dibujarcirculo(circles,frame)
	#copia_cir=circles
	#print type(copia_cir)

	#print "Circulos", len(circles)
	#rospy.sleep(2)
	#circles=np.round(circles[0,:].astype("int"))
	
	cv2.imwrite("Frame.jpg",frame)
	cv2.imshow("Frame",frame)
	cv2.waitKey(0)
	#send_image("Frame.jpg")
	#acercarse a la torre
	t=0.16
	#circles1= []
	#print type(circles1) 
	
	print "Dimension: ", circles.ndim
	#for i in range(circles.ndim):
	#	circles1.append(circles[i])
	#print circles1
	circles1=list(circles)
	print "circles1",circles1
	print "Tamaño Circle1: ",len(circles1)
	
	tamano=len(circles1)
	y=0.05
	