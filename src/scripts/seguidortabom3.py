#!/usr/bin/env python
# coding=utf-8



import rospy, cv2, cv_bridge, numpy, sys
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from random import randint
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
roll = None
pitch = None
yaw = None
altitudeZ = 0
integralZ = 0
last_erroZ = 0
distH = 0
vel_linear = 0
integral = 0
last_erro = 0

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
                self.r = 0 # Velocidade Angular (Yaw)
                self.r_atual = 0
                self.r_anterior = 0


                cv2.namedWindow("window", 1)


                self.image_sub = rospy.Subscriber('gazebo/camera/image_raw',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('/setpoint_offboard_TwstS',
                        TwistStamped, queue_size=1)

                self.twistS = TwistStamped()
                ##rospy.Subscriber('/camera/depth/image_raw',Image,self.get_distance)

                rospy.Subscriber('mavros/local_position/odom',Odometry,self.callback_odom)
                self.Odom = Odometry ()

                #self.otimiza = self.callback_client(self.omega,self.ze,self.thetar)

        def pidZ(self, erro, kp, ki, kd):
            global integralZ
            global last_erroZ
            integralZ += erro
            derivative = erro - last_erroZ
            last_erroZ = erro
            return kp * erro + ki * integralZ + kd * derivative

        def pid(self, erro, kp, ki, kd):
            global integral
            global last_erro
            integral += erro
            derivative = erro - last_erro
            last_erro = erro
            return kp * erro + ki * integral + kd * derivative





        def callback_client(self,roll,pitch,yaw,ze,phir,r,altitudeZ,vel_linear):
                rospy.wait_for_service('otimizador')
                try:
                        self.otimizador = rospy.ServiceProxy('otimizador', otimizador)
                        self.resposta = self.otimizador(roll,pitch,yaw,self.ze,self.phir,self.r,distH,vel_linear)
                        print('resposta:', self.resposta.u)
                        return self.resposta.u
                except rospy.ServiceException, e:
                        print "Falhou de novo haha: %s" %e


        def callback_odom(self,msg):
            global xr
            global yr
            global thr
            global roll
            global pitch
            global yaw  
            global tf
            global r
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
            global altitudeZ
            altitudeZ = msg.pose.pose.position.z
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
                    
                #rate = rospy.Rate(10)

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                lower_yellow = numpy.array([10, 10, 10])
                upper_yellow = numpy.array([255,255,250])
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask2 = cv2.inRange(hsv, lower_yellow, upper_yellow)

                ############################## Máscaras Para extração dos 3 pontos para calcular a curvatura da linha ################################################
                h, w, d = image.shape
                
                dist = 20
                search_top = 290 ## 450 (Eu mudei pra 290 então os valores comentados vão ta diferente)
                search_bot = search_top + dist ## 470
                mask[0:search_top, 0:w] = 0 ## 0 - 450 
                mask[search_bot:h, 0:w] = 0 ## 470 - 600

                search_top1 = search_top - dist ## 430
                search_bot1 = search_top1 + dist ## 450 
                mask1[0:search_top1, 0:w] = 0 ## 0 - 430
                mask1[search_bot1:h, 0:w] = 0 ## 450 - 600
                
                search_top2 = search_bot1 + dist ## 470
                search_bot2 = search_top2 + dist ## 490 
                mask2[0:search_top2, 0:w] = 0 ## 0 - 470
                mask2[search_bot2:h, 0:w] = 0 ## 490 - 600

                #As máscaras servem para limitar a extração dos centróides em uma determinada faixa de pixels (Faixas de altura h(600 pixels))
                M2 = cv2.moments(mask2)
                M1 = cv2.moments(mask1)
                M = cv2.moments(mask)

                if M['m00'] > 0 : ## Ponto Principal : Pr
                        
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    cv2.circle(image, (cx, cy), 5, (0,0,255), -1)

                    self.erro = cx - w/2 # Erro é o ponto do centróide em relação ao centro da câmera (w/2)
 

                if M1['m00'] > 0 : ## Ponto auxiliar P1
                        
                    cx1 = int(M1['m10']/M1['m00'])
                    cy1 = int(M1['m01']/M1['m00'])
                    cv2.circle(image, (cx1, cy1), 5, (255,0,0), -1)

                if M2['m00'] > 0 : # Ponto auxiliar P2
                    cx2 = int(M2['m10']/M2['m00'])
                    cy2 = int(M2['m01']/M2['m00'])
                    cv2.circle(image, (cx2, cy2), 5, (255,0,0), -1)
                ###########################################################################################################################
                   
                ## Cálculo da curvatura ###################################################################################################

                a = np.array([[cx1,cy1],[cx,cy],[cx2,cy2]]) # Ponto auxiliar 1 - Ponto Pr - Ponto Auxiliar 2
                

                #Cálculo das derivadas Primeiras
                dx_dt = np.gradient(a[:, 0])
                dy_dt = np.gradient(a[:, 1])
                velocity = np.array([ [dx_dt[i], dy_dt[i]] for i in range(dx_dt.size)]) ## Vetor Velocidade
                #print(velocity)
                ds_dt = np.sqrt(dx_dt * dx_dt + dy_dt * dy_dt) ## Vetor Aceleração

                tangent = np.array([1/ds_dt] * 2).transpose() * velocity ## Vetor Tangente Unitário

                tangent_x = tangent[:, 0]
                tangent_y = tangent[:, 1]

                deriv_tangent_x = np.gradient(tangent_x)
                deriv_tangent_y = np.gradient(tangent_y)

                dT_dt = np.array([ [deriv_tangent_x[i], deriv_tangent_y[i]] for i in range(deriv_tangent_x.size)])

                length_dT_dt = np.sqrt(deriv_tangent_x * deriv_tangent_x + deriv_tangent_y * deriv_tangent_y)

                normal = np.array([1/length_dT_dt] * 2).transpose() * dT_dt # Vetor Unitário Normal

                ## Cálculo das Derivadas segundas
                d2s_dt2 = np.gradient(ds_dt) 
                d2x_dt2 = np.gradient(dx_dt)
                d2y_dt2 = np.gradient(dy_dt)

                curvature = np.abs(d2x_dt2 * dy_dt - dx_dt * d2y_dt2) / (dx_dt * dx_dt + dy_dt * dy_dt)**1.5 ## Cálculo da Curvatura


                #################################################################################################################################


                vel_linear = 0.5 # Velocidade Linear do robô
                conv = 0.00125*2 #Conversão de metros 
                altitude = 1.5
                ##h1 = altitudeZ  # 0.5 metros
                distH = altitudeZ*np.tan(np.pi/4)
                print('disth',distH)
                self.ze = (self.erro)*(conv) # Z (metros) é igual ao erro (o quanto o centróide está distante do centro da câmera) multiplicado por um fator de conversão (pixel para metros)
                self.phir = np.arctan2(self.ze,distH) # ThetaR é igual a tangente entre o erro Z e a distância da câmera ao centróide (z) em Rad.

                self.r = self.r_anterior
                #print('ze',self.ze)
                #print ('thetar',self.thetar)
                ## ('omega',self.omega)



                self.otimiza = self.callback_client(roll,pitch,yaw,self.ze,self.phir,self.r,distH,vel_linear) # Manda as variáveis de interesse para o otimizador.
                uoptimal = self.resposta.u ## A variavel de entrada u é recebida pela variável resposta.u .

                self.r_atual = (uoptimal*np.cos(self.phir) + curvature[1]*np.cos(pitch)*np.cos(yaw)*vel_linear) / ((np.cos(self.phir) - curvature[1]*self.ze)*(np.cos(roll)/np.cos(pitch))) # Cálculo do omega que será aplicado ao robô
                ## Multiplicar por 3 melhorou o controle - Provavelmente se aumentar o ganho da saída no otimizador consiga melhores resultados.
                ## Adicionar as parcelas referentes a velocidade em y .
                self.twistS.twist.linear.x = vel_linear*np.cos(yaw)
                self.twistS.twist.linear.y = vel_linear*np.sin(yaw)
                #print(self.twistS.twist.linear.x)
                self.twistS.twist.angular.z = self.pid(self.r_atual, 3, 0.1, 0.0)
                self.r_anterior = self.r_atual   
                print(self.ze)
     

                #self.twist.linear.x = 0
                #self.twist.angular.z = 0
                d_altitude = 1.5 - altitudeZ
                self.twistS.twist.linear.z = self.pidZ(d_altitude, 0.5, 0.0001, 0.1)
                self.cmd_vel_pub.publish(self.twistS)
                            
                
                ##self.cmd_vel_pub.publish(self.twist)

                cv2.imshow("window", image)
                cv2.imshow("window1",mask)
                #cv2.imshow("window1",mask1)

                cv2.waitKey(3)

rospy.init_node('line_follower')
follower = Follower()
rospy.spin()
