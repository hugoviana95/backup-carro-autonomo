#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2 # OpenCV2 for saving an image
import numpy as np
import math
import threading
import os
import time
from PIL import Image as I
import array
import time
#import cv
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# ROS float message
from std_msgs.msg import Float32
# ROS Bool message
from std_msgs.msg import Bool



# Normalizando os dados do estressamento de 45 e -45 para 1 e -1
Nmin=-1
Nmax=1
Xmax=45
Xmin=-45

#criar pasta de teste
dir = './teste1'  # cria pasta de testes com localização e data
os.mkdir(dir)

# criamos pgastas para guardar as imaens para cada camera
can1 = './teste1/Camera_Depth'
os.mkdir(can1)
can2 = './teste1/Camera_RGB'
os.mkdir(can2)

# Define  a cameras que serã usadas na captura multicameras
Can1 = 0
Can2 = 1
x1 = 0
s = " "  # separação de itens salvos
data_N = 0
data_DN = 0
cv2_img2 = 0
cv2_img1 = 0
steering_angle_degree1 = 0
steering_angle_degree = 0
steering_angle_Norma = 0

im=0

# Instantiate CvBridge
bridge = CvBridge()

def tempo():  # definimos o tempo de espera para pegar dados
    temporizador=rospy.sleep(0.4) # 10 segundos
    return(temporizador)

def leftSter_callback(msg): # Aqui estamos subtraindo a posição da roda esquerda e seguidamente é feito o calculo para definir a media total do estressamento das rodas da frente (steering_angle)
    global steering_angle_Norma,steering_angle_degree1,steering_angle_degree

    #print("Received Angle!")
    left_steer = msg.data
   #rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
   #print(left_steer)
   # Distancia entre as rodas da esquerda e direita do carro
    l = 1.5103
   # Distancia entre as rodas dianteiras e traseiras do carro
    d = 2.5772
    if left_steer > 0:
           R = d * np.tan(math.pi / 2 - left_steer) + l / 2
    else:
           R = -(d * np.tan(math.pi / 2 - abs(left_steer)) - l / 2)

    steering_angle = np.arctan((1 / R) * d) # media do estresamento das rodas da frente
    steering_angle_degree = ((steering_angle * 180)/math.pi) * 2  # aqui colocamos os dados em graus
    #print(steering_angle_degree)

    print(steering_angle)
    #return (steering_angle_Norma)


def image_callback1(msg):  # Pegamos Imagens de profundidade
    global cv2_img1

    #print("Received an image Depht!")

    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        cv2_img = np.rot90(cv2_img, 2)  # rotaciona 2 vezes
        cv2_img1 = np.fliplr(cv2_img)  # inverte colunas por filas
        # Save your OpenCV2 image as a png
        #cv2.imwrite('image_Depth.png', cv2_img)
        #return(cv2_img)
       # Definimos tamanho e onde será gravado cada frame das cameras


def image_callback2(msg): # Pegamos imagens RGB
    global cv2_img2

    #print("Received an image RGB!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        cv2_img = np.rot90(cv2_img, 2)  # rotaciona 2 vezes
        cv2_img2 = np.fliplr(cv2_img)  # inverte colunas por filas
        # Save your OpenCV2 image as a png
        #cv2.imwrite('image_RGB.png', cv2_img)



def save_data(event):
    global cv2_img2,cv2_img1,steering_angle_degree, steering_angle_degree1,steering_angle_Norma,s,x1,Xmin,Xmax,Nmin,Nmax,data_N,data_DN,im,steering_angle

    PastaFrame1 = 'teste1/Camera_Depth'
    nomeDoFrame1 = PastaFrame1 + '/%d.png' % x1

#    im = cv2.imread(cv2_img1)
 #   print(im)
    #cv2_img11= cv2.cvtColor(cv2_img1, cv2.COLOR_BGR2GRAY)
    img_scaled1 =  cv2.resize( cv2_img1,(455, 256))  # Definimos tamanho dos frames 1  #  original (455, 256) , default do kinect (640, 480)
    cv2.imwrite(nomeDoFrame1, img_scaled1)  # salvamos frames

    PastaFrame2 = 'teste1/Camera_RGB'
    nomeDoFrame2 = PastaFrame2 + '/%d.png' % x1

    img_scaled2 = cv2.resize( cv2_img2, (455, 256), interpolation=cv2.INTER_AREA )  # Definimos tamanho dos frames 1  #  original (455, 256) , default do kinect (640, 480)
    cv2.imwrite(nomeDoFrame2, img_scaled2)  # salvamos frames

    nomeDoFrame1x = '%d.png' % x1
    steering_angle_degree1 = str("%.6f" % (steering_angle_degree * 10))                 #str(round(steering_angle_degree, 7))
    steering_angle_Norma = (((((steering_angle_degree) - Xmin) * (Nmax - Nmin)) / (Xmax - Xmin)) + Nmin) # Equação de Normalização do PWM para (-1 a 1)
    steering_angle_Norma1 = str("%.6f" % steering_angle_Norma)                    #str(round(steering_angle_Norma, 7))
    #print(steering_angle_degree1)

    data_DN = nomeDoFrame1x + s + (steering_angle_degree1)  # Definemos a posição de cada dado no arquivo data.txt
    data_N = nomeDoFrame1x + s + steering_angle_Norma1  # Definemos a posição de cada dado no arquivo data_N.txt

    arquivo = open('teste1/data_DN.txt','a')  # para abrir arquivo "data" e o "a" representa adição de novos dados no final da fila (Sem normalização)
    arquivo1 = open('teste1/data_N.txt','a')  # para abrir arquivo "data_N" e o "a" representa adição de novos dados no final da fila (Com nornamlização)

    arquivo.write('%s\n' % data_DN)  # Escrevemos no arquivo data.txt
    arquivo1.write('%s\n' % data_N)  # Escrevemos no arquivo data_N.txt

    x1 += 1
    print(steering_angle_degree1)


def listener():
  #global x1
  #global s
  # In ROS, nodes are uniquely named. If two nodes with the same
  # node are launched, the previous one is kicked off. The
  # anonymous=True flag means that rospy will choose a unique
  # name for our 'listener' node so that multiple listeners can
  # run simultaneously.

  ########## Aqui Mandamos as variaveis via ROS ###############
  pub_EnableSyncMode = rospy.Publisher('enableSyncMode', Bool, queue_size=1)
  pub_StartSim = rospy.Publisher('startSimulation', Bool, queue_size=1)
  #pub_StopSim = rospy.Publisher('stopSimulation', Bool, queue_size=1)
  pub_leftMotor = rospy.Publisher('leftMotorSpeed', Float32, queue_size=1)
  pub_rightMotor = rospy.Publisher('rightMotorSpeed', Float32, queue_size=1)


  rospy.init_node('listener', anonymous=True) #inicializamos pos topicos

  # pub_EnableSyncMode.publish(False)

  pub_EnableSyncMode.publish(False)
  start_time = time.time()
  end_time = time.time()
  while (end_time - start_time) < 0.2:
      pub_StartSim.publish(True)
      # pub_EnableSyncMode.publish(False)
      # pub_TriggerNextStep.publish(True)
      end_time = time.time()


  velocity_input = 6 # 2.8
  left_motor = Float32()
  right_motor = Float32()
  r = 6.3407e-01 / 2.0
  left_motor.data = velocity_input / r
  right_motor.data = velocity_input / r
  pub_rightMotor.publish(right_motor)
  pub_leftMotor.publish(left_motor)
  #print('velocidade: ', left_motor)

  ########## Aqui Aquirimos as variaveis via ROS ###############
  rospy.Subscriber("leftCANSteeringAngle", Float32, leftSter_callback)
  #rospy.init_node('image_listener')
  # Define your Depth image
  image_topic1 = "/image1"
  # Define your RGB image
  image_topic2 = "/image2"
  # Set up your subscriber and define its callback
  rospy.Subscriber(image_topic1, Image, image_callback1)
  # Set up your subscriber and define its callback
  rospy.Subscriber(image_topic2, Image, image_callback2)

  rospy.Timer(rospy.Duration(0.1), save_data)

  #tempo()

  rospy.spin()

if __name__ == '__main__':

    listener()
  #  savedata()

