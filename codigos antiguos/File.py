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
# Pose inicial
xx = 0.6
yy = -0.3
zz = 0.17 
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
cam_calibracion = 0.0025            # 0.0025 pixeles por metro a 1 metro de distancia. Factor de correccion
cam_x_offset    = 0.04              # Correccion de camara por los gripper,
cam_y_offset    = -0.015       
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
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #tranformacion de la imagen a escla de grises
	dimensiones =gray.shape
	kernel = np.ones((3,3),np.uint8)
	gray = cv2.GaussianBlur(gray,(5,5),0)#para eliminar el ruido de la imagen
	gray = cv2.medianBlur(gray,5) #reduce el ruido de sal
	gray = cv2.erode(gray,kernel,iterations = 1)#erosiona la imagen
	gray = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,3.5)
	
	#detectar circulos en la imagen
	circles = cv2.HoughCircles(gray, cv.CV_HOUGH_GRADIENT, 1, 200, param1=40, param2=45, minRadius=20, maxRadius=140)
	return circles 

def dibujarcirculo(circles,frame):
	print "circles" , circles
	if circles is not None:
		# convert the (x, y) coordinates and radius of the circles to integers
        
		circles = np.round(circles[0, :]).astype("int")
		print circles[0][0]
		print circles[0][1]
		# loop over the (x, y) coordinates and radius of the circles
		for (x, y, r) in circles:	
			cv2.circle(frame, (x, y), r, (0, 255, 0), 4) #dibujar circulo periferico
			cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1) #dibujar rectangulo en el centro
			

#right_camera_callback(data)
rospy.Subscriber('/cameras/right_hand_camera/image', Image, callback)

#while not rospy.is_shutdown():
circle	=[]
while not rospy.is_shutdown():	# Capture frame-by-frame
	while np.all(foto) == None:
		print "Ahorita no joven"
		continue
	frame=foto
	circles=transformacion(frame)
	frame1=dibujarcirculo(circles,frame)
	copia_cir=circles
	print type(copia_cir)

	print "Circulos", len(circles)
	rospy.sleep(2)
	circles=np.round(circles[0,:].astype("int"))
	cv2.imwrite("Frame.jpg",frame1)
	#send_image("Frame.jpg")
	#acercarse a la torre
	i=0.17
	while len(circles) :
		
		puntox=circles[0][0]
		puntoy=circles[0][1]
		
		print puntox , puntoy
		punto=pixel_to_baxter((puntox,puntoy),0.17)
		print punto
		rospy.sleep(0.5)
		mover_baxter('base',[punto[0],punto[1],0.17],[math.pi,0,0])
		mover_baxter('base',[punto[0],punto[1],-0.17],[math.pi,0,0])
		gripper.close()
		rospy.sleep(2)
		mover_baxter('base',[punto[0],punto[1],0.17],[math.pi,0,0])
		mover_baxter('base',[punto[0]-0.4,punto[1]+0.4,0.17],[math.pi,0,0])
		mover_baxter('base',[punto[0]-0.4,punto[1]+0.4,-i],[math.pi,0,0])
		rospy.sleep(1)
		gripper.open()
		rospy.sleep(2)
		mover_baxter('base',[punto[0]-0.4,punto[1]+0.4,0.17],[math.pi,0,0])
		mover_baxter('base',[0.9,-0.2,0.17],[math.pi,0,0])
i=i-0.03