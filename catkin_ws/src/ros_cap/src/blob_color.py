#!/usr/bin/env python


import rospy
# import rospkg
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse

from cv_bridge import CvBridge, CvBridgeError

import numpy as np

# define range of blue color in HSV
lower_blue = np.array([110,50,50])
upper_blue = np.array([130,255,255])
lower_red = np.array([0,0,0])
upper_red = np.array([0,0,0])
lower_yellow = np.array([25,170,50])
upper_yellow = np.array([30,255,255])


class BlobColor():

    def __init__(self):


        #Subscribirce al topico "/duckiebot/camera_node/image/raw"
        self.image_subscriber = rospy.Subscriber('/duckiebot/camera_node/image/raw', Image, self._process_image)
        self.base_pub = rospy.Publisher('/duckiebot/segment_node/image', Image, queue_size=1)
        self.base_pub_2 = rospy.Publisher('/duckiebot/geometry_msgs/Punto', Point, queue_size=1)
        #Clase necesaria para transformar el tipo de imagen
        self.bridge = CvBridge()

        #Ultima imagen adquirida
        self.cv_image = Image()

        self.min_area = 20



    def _process_image(self,img):
        #Se cambiar mensage tipo ros a imagen opencv
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        #Se deja en frame la imagen actual
        frame = self.cv_image

        #Cambiar tipo de color de BGR a HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Filtrar colores de la imagen en el rango utilizando 
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        # segment_image = cv2.bitwise_and(frame,frame, mask= mask)


        kernel = np.ones((5,5),np.uint8)

        #Operacion morfologica erode
        img_out = cv2.erode(mask, kernel, iterations = 1)
        
        #Operacion morfologica dilate
        img_out = cv2.dilate(img_out, kernel, iterations = 1)
        img_out = cv2.erode(img_out, kernel, iterations = 1)
        img_out = cv2.dilate(img_out, kernel, iterations = 1)

        image, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        x1,y1,w1,h1 = 0,0,0,0
        for cnt in contours:
            #Obtener rectangulo
            x,y,w,h = cv2.boundingRect(cnt)
            

            #Filtrar por area minima
            if w*h > self.min_area:

                #Dibujar un rectangulo en la imagen
                cv2.rectangle(frame, (x,y), (x+w,y+h), (0,0,0), 2)
            if w*h > w1*h1:
                x1,y1,w1,h1 = cv2.boundingRect(cnt)
                

        xc, yc= (x1+w1)/2, (y1+h1)/2
        
        imagen = self.bridge.cv2_to_imgmsg(frame, "bgr8")

        self.base_pub.publish(imagen)#Publicar frame

        #Publicar Point center de mayor tamanio
        self.base_pub_2.publish(xc,yc,0)

def main():

    rospy.init_node('BlobColor')

    BlobColor()

    rospy.spin()

if __name__ == '__main__':
    main()
