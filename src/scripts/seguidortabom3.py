#!/usr/bin/env python
# coding=utf-8



import rospy, cv2, cv_bridge, numpy, sys
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
from roslib import message
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from nmpc_robo.srv import *
xr=None
yr=None
thr=None
z = None

# Funcionou para mundotrab3.world
# mundo4.world ta bom tbm
#mundo5.world ta bom pros 2 lados

class Follower:

        



        def __init__(self):
                self.angulorobo = 0
                self.soma = 0
                self.z = 0
                self.lastz = 0
                self.flag = 0
                self.flag1 = 0
                self.lastrho = 0
                self.aux = 0
                self.xfut = 0
                self.yfut = 0
                self.cont = 0
                self.erro = 0
                self.minimo = 0
                self.bridge = cv_bridge.CvBridge()
                cv2.namedWindow("window", 1)

                self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                        Twist, queue_size=1)

                self.twist = Twist()
                rospy.Subscriber('/camera/depth/image_raw',Image,self.get_distance)

                rospy.Subscriber('/odom',Odometry,self.callback_odom)
                self.omega = 3.3
                self.ze = 2.2
                self.thetar = 5.7
                print('entrou aqui 1')
                self.otimiza = self.callback_client(self.omega,self.ze,self.thetar)

                
        def callback_client(self,omega,ze,thetar):
                rospy.wait_for_service('otimizador')
                try:
                        self.otimizador = rospy.ServiceProxy('otimizador', otimizador)
                        resposta = self.otimizador(omega,ze,thetar)
                        print('resposta:', resposta.u)
                        return resposta.u
                except rospy.ServiceException, e:
                        print "Falhou caralho: %s" %e
                        
                rospy.spin()


        def callback_odom(self,msg):
            global xr
            global yr
            global thr
            global roll
            global pitch
            global yaw  
            global tf
            global omega
            global ze
            global thetar


            
            xr=msg.pose.pose.position.x
            yr=msg.pose.pose.position.y
            thr = np.arctan2(2*msg.pose.pose.orientation.w*msg.pose.pose.orientation.z,1-2*msg.pose.pose.orientation.z*msg.pose.pose.orientation.z) 

            

            quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
            )
            euler = euler_from_quaternion(quaternion)
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]
                


        def get_distance(self,img):
                
                self.bridge = cv_bridge.CvBridge()
                cv_image = self.bridge.imgmsg_to_cv2(img,'32FC1')
                cv_image = np.array(cv_image, dtype=np.float32)
                
                #print ('dist',cv_image)
                return cv_image

              
        def image_callback(self, msg):
                
                
                fat = 0
                
                ptime = 0
                frep = []
                ftot = 0
                krep = 6
                distobs1 = []
                katt = 2
                rho = 0

                
                #rate = rospy.Rate(10)

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                lower_yellow = numpy.array([10, 10, 10])
                upper_yellow = numpy.array([255,255,250])
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

                h, w, d = image.shape
                
                search_top = 3*h/4
                search_bot = search_top + 300
                mask[0:search_top, 0:w] = 0
                mask[search_bot:h, 0:w] = 0
                
                    
                

                M = cv2.moments(mask)
                #print('M',M)
                if M['m00'] > 0 and self.flag1 == 0:
                        
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

                    fatt = cx - w/2
                    self.erro = cx - w/2
                    #print('fat',fatt)

                    self.z = float(fatt) / 600
                    #print ('fatt',self.z)
                    ftot = self.z
                    #print('frep')  
                    #self.twist.linear.x = 0
                    #self.twist.angular.z = 0

                    #gamarep = np.arctan2(frep[1],frep[0]) 
                    self.twist.linear.x = 0.6
                    self.twist.angular.z = ftot                


                    
                

                

                dt = rospy.get_time() - ptime
                #self.lastz = self.z
                #ftot = self.z + ((self.z - self.lastz)/(dt)*0.5)+1.5*gamarep #estático 1.1
                


                #self.twist.linear.x = 0
                #self.twist.angular.z = 0
                ptime = rospy.get_time()


                            
                
                self.cmd_vel_pub.publish(self.twist)

                cv2.imshow("window", image)
                cv2.imshow("window1",mask )

                cv2.waitKey(3)

rospy.init_node('line_follower')
follower = Follower()
rospy.spin()

