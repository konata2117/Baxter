#!/usr/bin/env python
import numpy as np
import argparse
import cv2
import time
import roslib
import rospy
import baxter_interface
import math
import tf
import cv2.cv as cv
import cv_bridge
from sensor_msgs.msg import Image
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

class mover():
#Function Start
	foto = None
	def __init__(self, arm):

		# Brazo a utilizar
		self.limb           = arm
		self.limb_interface = baxter_interface.Limb(self.limb)

		if arm == "left":
			self.other_limb = "right"
		else:
			self.other_limb = "left"
	   
		self.other_limb_interface = baxter_interface.Limb(self.other_limb)       

		# set speed as a ratio of maximum speed
		self.limb_interface.set_joint_position_speed(1)
		self.other_limb_interface.set_joint_position_speed(1)
	
		'''
		# Gripper ("left" or "right")
		self.gripper = baxter_interface.Gripper(arm)

		# calibrate the gripper
		self.gripper.calibrate()

		# Parametros de camara
		self.cam_calibracion = 0.0025            # 0.0025 pixeles por metro a 1 metro de distancia. Factor de correccion
		self.cam_x_offset    = 0.04              # Correccion de camara por los gripper,
		self.cam_y_offset    = -0.015       
		self.resolution      = 1
		self.width           = 960               # 1280 640  960
		self.height          = 600               # 800  400  600
		'''
		# Abrir Camara
		self.open_camera(self.limb, self.width, self.height)
		# Pose inicial
		self.x = 0.6
		self.y = -0.3
		self.z = -0.1 
		self.roll = math.pi	#Rotacion x
		self.pitch = 0.0	#Rotacion y	
		self.yaw = 0.0		#Rotacion z
		self.pose_i = [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]
		self.pose = [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]

		#self.mover_baxter('base',self.pose_i[:3],self.pose_i[3:6])
		#Function End
		'''
		rospy.Subscriber('/cameras/right_hand_camera/image', Image , self.callback)
	
		# reset all cameras (incase cameras fail to be recognised on boot)
	def reset_cameras(self):
		reset_srv = rospy.ServiceProxy('cameras/reset', std_srvs.srv.Empty)
		rospy.wait_for_service('cameras/reset', timeout=10)
		reset_srv()
	'''
	# Abrir camras y setear parametros
	def open_camera(self, camera, x_res, y_res):
		if camera == "left":
			cam = baxter_interface.camera.CameraController("left_hand_camera")
		elif camera == "right":
			cam = baxter_interface.camera.CameraController("right_hand_camera")
		elif camera == "head":
			cam = baxter_interface.camera.CameraController("head_camera")
		else:
			sys.exit("ERROR - open_camera - Invalid camera")

		# close camera
		#cam.close()

		# set camera parameters
		cam.resolution          = cam.MODES[0]
		#cam.exposure            = 30             # range, 0-100 auto = -1
		#cam.gain                = -1             # range, 0-79 auto = -1
		#cam.white_balance_blue  = -1             # range 0-4095, auto = -1
		#cam.white_balance_green = -1             # range 0-4095, auto = -1
		#cam.white_balance_red   = -1             # range 0-4095, auto = -1

		# open camera
		cam.open()
	
	# convert Baxter point to image pixel
	def baxter_to_pixel(self, pt, dist):
		x = (self.width / 2)                                                         \
		  + int((pt[1] - (self.pose[1] + self.cam_y_offset)) / (self.cam_calibracion * dist))
		y = (self.height / 2)                                                        \
		  + int((pt[0] - (self.pose[0] + self.cam_x_offset)) / (self.cam_calibracion * dist))

		return (x, y)

	# convert image pixel to Baxter point
	def pixel_to_baxter(self, px, dist):
		x = ((px[1] - (self.height / 2)) * self.cam_calibracion * dist)                \
		  + self.pose_i[0] + self.cam_x_offset
		y = ((px[0] - (self.width / 2)) * self.cam_calibracion * dist)                 \
		  + self.pose_i[1] + self.cam_y_offset

		return (x, y)
	'''
	# close a camera
	def close_camera(self, camera):
		if camera == "left":
			cam = baxter_interface.camera.CameraController("left_hand_camera")
		elif camera == "right":
			cam = baxter_interface.camera.CameraController("s_hand_camera")
		elif camera == "head":
			cam = baxter_interface.camera.CameraController("head_camera")
		else:
			sys.exit("ERROR - close_camera - Invalid camera")

		# set camera parameters to automatic
		cam.exposure            = -1             # range, 0-100 auto = -1
		cam.gain                = -1             # range, 0-79 auto = -1
		cam.white_balance_blue  = -1             # range 0-4095, auto = -1
		cam.white_balance_green = -1             # range 0-4095, auto = -1
		cam.white_balance_red   = -1             # range 0-4095, auto = -1

		# close camera
		cam.close()

	'''
	def callback(self,msg):
		# Transforma el mensaje a imagen
		try:
			self.foto = cv_bridge.CvBridge().imgmsg_to_cv2(msg) #, "bgr8") #bgr8
		except cv_bridge.CvBridgeError, e:
			print e	   


	def imagen(self):
		rospy.Subscriber('/cameras/left_hand_camera/image', Image, callback)

		while not rospy.is_shutdown():
			# Capture frame-by-frame
			while np.all(foto) == None:
				print "Ahorita no joven"
				continue 

			#ret, frame = cap.read()
			frame = foto
			#load the image, clone it for output, and then convert it to grayscale
			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			dimensiones = gray.shape
			RGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
			cv2.imshow("RGB",RGB)
			kernel = np.ones((3,3),np.uint8)
			gray = cv2.GaussianBlur(gray,(5,5),0);
			gray = cv2.medianBlur(gray,5)
			gray = cv2.erode(gray,kernel,iterations = 1)
			gray = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,3.5)
			#detectar circulos en la imagen
			circles = cv2.HoughCircles(gray, cv.CV_HOUGH_GRADIENT, 1, 200, param1=40, param2=45, minRadius=20, maxRadius=140)
			# preguntar si los circulos fueron encontrados
			if circles is not None:
			   # convert the (x, y) coordinates and radius of the circles to integers
			   print "circles" , circles
			   circles = np.round(circles[0, :]).astype("int")
			   # loop over the (x, y) coordinates and radius of the circles
			   for (x, y, r) in circles:
					print x , y , r
					# draw the circle in the output image, then draw a rectangle in the image
					# corresponding to the center of the circle
					cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
					cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
					time.sleep(0.5)
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
		return circles    
	   
	

	def mensaje_matriz_a_pose(self,T, frame):
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

	def mover_baxter(self, source_frame, trans, rot):

		nombre_servicio = '/ExternalTools/'+ self.limb +'/PositionKinematicsNode/IKService'
		servicio_ik = rospy.ServiceProxy(nombre_servicio,SolvePositionIK)
		frame = source_frame   

		# Promedio de velocidad del brazo
		self.limb_interface.set_joint_position_speed(0.1)

		matrix = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans),
			tf.transformations.euler_matrix(rot[0],rot[1],rot[2])
			)
		
		rospy.wait_for_service(nombre_servicio,10)
		ik_mensaje = SolvePositionIKRequest()
		ik_mensaje.pose_stamp.append(self.mensaje_matriz_a_pose(matrix, frame))

		try:
			respuesta = servicio_ik(ik_mensaje)
		except:
			print "Movimiento no ejecutado"

		print respuesta.isValid[0]

		if respuesta.isValid[0] == True:
			movimiento =  dict(zip(respuesta.joints[0].name, respuesta.joints[0].position))
			self.limb_interface.move_to_joint_positions(movimiento)
		else:
			print "Movimiento no ejecutado"

		print respuesta.joints[0].position
		print respuesta.joints[0].name


def main():
	rospy.init_node('Movimiento', anonymous = True) #anonimo para que no tenga alcance de nombres

	mov = mover('left')
	mov.callback()
	print mov.image()
	mov.mover_baxter('base',[0.69, -0.48, -0.03],[-3.09,-0.095,-3.08])
	mov.mover_baxter('base',[0.69, 0.18, -0.03],[-3.09,-0.095,-3.08])

if __name__ == "__main__":
	main()
