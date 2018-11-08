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

cap = baxter_interface.CameraController('left_hand_camera') 
cap.open()

cap.resolution = cap.MODES[0]

foto = None
# Pose inicial
xx = 0.6
yy = 0.3
zz = 0.1 
roll = math.pi	#Rotacion x
pitch = 0.0	#Rotacion y	
yaw = 0.0		#Rotacion z

pose_i = [xx, yy, zz,roll,pitch, yaw]
pose = [xx,yy,zz,roll, pitch, yaw]
gripper = baxter_interface.Gripper("left")
limb_interface = baxter_interface.Limb('left')
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
	nombre_servicio = '/ExternalTools/'+ 'left' +'/PositionKinematicsNode/IKService'
	servicio_ik = rospy.ServiceProxy(nombre_servicio,SolvePositionIK)
	frame = source_frame   

	# Promedio de velocidad del brazo
	limb_interface.set_joint_position_speed(0.1)

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
rospy.Subscriber('/cameras/left_hand_camera/image', Image, callback)

while not rospy.is_shutdown():
	# Capture frame-by-frame
	while np.all(foto) == None:
		print "Ahorita no joven"
		continue 

	#ret, frame = cap.read()
 
	frame= foto
	# load the image, clone it for output, and then convert it to grayscale
			

	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	dimensiones =gray.shape
        RGB=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
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
		print circles[0][0]
		print circles[0][1]
		# loop over the (x, y) coordinates and radius of the circles
		for (x, y, r) in circles:
                        print x , y , r
			# draw the circle in the output image, then draw a rectangle in the image
			# corresponding to the center of the circle
			cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
			cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
			cv2.imshow('gray',gray)
			xx=pixel_to_baxter((circles[0][0],circles[0][1]),0.32)
			print xx[0], xx[1]
			mover_baxter('base',[xx[0], xx[1], -0.03],[-3.09,-0.095,-3.08])
			#yy=pixel_to_baxter(y,0,3)
			#print "Coordenada es: ", xx
			#print "Column Number: "
			#print x
			#print "Row Number: "
			#print y
			print "Radius is: "
			print r

	# Display the resulting frame
        
    	cv2.imshow('frame',frame)

 	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# When everything done, release the capture

cv2.destroyAllWindows()
