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
ptime = 0
last_erro = 0
dt = 0.1
font = cv2.FONT_HERSHEY_COMPLEX


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
                self.ptime = 0
                self.bridge = cv_bridge.CvBridge()
                self.r = 0 # Velocidade Angular (Yaw)
                self.r_atual = 0
                self.r_anterior = 0
                self.integral = 0
                self.last_erro = 0
                self.derivative = 0
                


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

        def pid(self, erro, kp, ki, kd,dt):

            self.integral += erro
            self.integral = integral*dt
            self.derivative = (erro - last_erro) / dt
            self.last_erro = erro
            return kp * erro + ki * self.integral + kd * self.derivative
        
        def get_curva(self,x1,y1,x2,y2,x3,y3):

                denom = (x1 - x2) * (x1 - x3) * (x2 - x3)
                denom = float(denom)
                A     = (x3 * (y2 - y1) + x2 * (y1 - y3) + x1 * (y3 - y2)) / denom
                B     = (x3*x3 * (y1 - y2) + x2*x2 * (y3 - y1) + x1*x1 * (y2 - y3)) / denom
                C     = (x2 * x3 * (x2 - x3) * y1 + x3 * x1 * (x3 - x1) * y2 + x1 * x2 * (x1 - x2) * y3) / denom
                A = float(A)
                B = float(B)
                C = float (C)
                #print ('A',A)
                #print ('B',B)
                #print ('C',C)
                #print ('denom',denom)
                x300 = A*300*300 + B*300 + C
                x280 = A*280*280 + B*280 + C
                x320 = A*320*320 + B*320 + C
                x300 = int(x300)
                x280 = int(x280)
                x320 = int(x320)
                return x300,x280,x320 


        # mask[0] : largura (800)
        # mask : comprimento (600) 
        def tamanho_linha (self,mask,image):
                _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if len(contours) > 0 :
                        for cnt in contours:
                                approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
                                #print(len(approx))
                                #cv2.drawContours(image, [approx], 0, (0), 5)
                                x = approx.ravel()[0]
                                y = approx.ravel()[1]

                                box = cv2.boundingRect(cnt)
                                #print(len(contours))
                                tamanho_janela = box[3]/3                      
                else :
                        tamanho_janela = 1


                return tamanho_janela





        def callback_client(self,roll,pitch,yaw,ze,phir,r,altitudeZ,vel_linear):
                rospy.wait_for_service('otimizador')
                try:
                        self.otimizador = rospy.ServiceProxy('otimizador', otimizador)
                        self.resposta = self.otimizador(roll,pitch,yaw,self.ze,self.phir,self.r,distH,vel_linear)
                        #print('resposta:', self.resposta.u)
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
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow) # Pr
                mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow) # P1
                mask2 = cv2.inRange(hsv, lower_yellow, upper_yellow) # P2
                mask3 = cv2.inRange(hsv, lower_yellow, upper_yellow) # PCurvaSuperior1
                mask4 = cv2.inRange(hsv, lower_yellow, upper_yellow) # PCurvaSuperior2
                mask5 = cv2.inRange(hsv, lower_yellow, upper_yellow) # PCurvaSuperior3
                mask6 = cv2.inRange(hsv, lower_yellow, upper_yellow) # PCurvaInferior1
                mask7 = cv2.inRange(hsv, lower_yellow, upper_yellow) # PCurvaInferior2
                mask8 = cv2.inRange(hsv, lower_yellow, upper_yellow) # PCurvaInferior3
                mask9 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask10 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                #print(mask9)





                #cv2.imshow("shapes",image)
                #cv2.imshow("Threshold", mask)





                ############################## Máscaras Para extração dos 3 pontos para calcular a curvatura da linha ################################################
                h, w, d = image.shape

                search_top9 = 0 ## 450 (Eu mudei pra 290 então os valores comentados vão ta diferente)
                search_bot9 = 300 ## 470
                mask9[0:search_top9, 0:w] = 0 ## 0 - 450 
                mask9[search_bot9:h, 0:w] = 0 ## 470 - 600

                search_top10 = 300 ## 450 (Eu mudei pra 290 então os valores comentados vão ta diferente)
                search_bot10 = 600 ## 470
                mask10[0:search_top10, 0:w] = 0 ## 0 - 450 
                mask10[search_bot10:h, 0:w] = 0 ## 470 - 600

                
                dist = 20

                janela_superior = int(self.tamanho_linha(mask9,image))
                janela_inferior = int(self.tamanho_linha(mask10,image))
                #print (janela_inferior)
                ### Pontos do meio

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

                ###############################

                ################## Pontos após a falha

                search_top3 = 0 
                search_bot3 = janela_superior ## 
                mask3[0:search_top3, 0:w] = 0 ## 0 - 470
                mask3[search_bot3:h, 0:w] = 0 ## 490 - 600

                search_top4 = janela_superior ## 470
                search_bot4 = 2*janela_superior ## 490 
                mask4[0:search_top4, 0:w] = 0 ## 0 - 470
                mask4[search_bot4:h, 0:w] = 0 ## 490 - 600

                search_top5 = 2*janela_superior ## 470
                search_bot5 = 3*janela_superior ## 490 
                mask5[0:search_top5, 0:w] = 0 ## 0 - 470
                mask5[search_bot5:h, 0:w] = 0 ## 490 - 600

                ##### Pontos Antes da falha

                search_top6 = 600 - 3*janela_inferior 
                search_bot6 = 600-  2*janela_inferior ## 
                mask6[0:search_top6, 0:w] = 0 ## 0 - 470
                mask6[search_bot6:h, 0:w] = 0 ## 490 - 600

                search_top7 = 600 - 2*janela_inferior ## 470
                search_bot7 = 600 - janela_inferior ## 490 
                mask7[0:search_top7, 0:w] = 0 ## 0 - 470
                mask7[search_bot7:h, 0:w] = 0 ## 490 - 600

                search_top8 = 600 - janela_inferior ## 470
                search_bot8 = 600 ## 490 
                mask8[0:search_top8, 0:w] = 0 ## 0 - 470
                mask8[search_bot8:h, 0:w] = 0 ## 490 - 600

                #As máscaras servem para limitar a extração dos centróides em uma determinada faixa de pixels (Faixas de altura h(600 pixels))

                M8 = cv2.moments(mask8)
                M7 = cv2.moments(mask7)
                M6 = cv2.moments(mask6)
                M5 = cv2.moments(mask5)
                M4 = cv2.moments(mask4)
                M3 = cv2.moments(mask3)
                #### Ponto do meio
                M2 = cv2.moments(mask2)
                M1 = cv2.moments(mask1)
                M = cv2.moments(mask)
                flag = 1
                #print ('janela_inferior',janela_inferior)
                #print ('janela_superior',janela_superior)
                #print ('flag',flag)
                if janela_superior < janela_inferior :
                        flag = 1
                else : # janela superior > janela inferior
                        flag = 0

                if M['m00'] > 0 and M1['m00'] > 0 and M2['m00'] > 0  : ## Encontrou o Ponto Principal : Pr

                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        cv2.circle(image, (cx, cy), 5, (0,0,255), -1)
                        self.erro = cx - w/2 # Erro é o ponto do centróide em relação ao centro da câmera (w/2)

                
                        cx1 = int(M1['m10']/M1['m00'])
                        cy1 = int(M1['m01']/M1['m00'])
                        cv2.circle(image, (cx1, cy1), 5, (255,0,0), -1)

                        
                        cx2 = int(M2['m10']/M2['m00'])
                        cy2 = int(M2['m01']/M2['m00'])
                        cv2.circle(image, (cx2, cy2), 5, (255,0,0), -1)

                        a = np.array([[cx1,cy1],[cx,cy],[cx2,cy2]]) # Pontos para o cálculo da curvatura 

                elif M3['m00'] > 0 and M7['m00'] > 0 and M8['m00'] > 0 and flag == 1  : ## Caso não encontrou a linha no ponto central e encontrou pontos antes e depois da falha - ENTRAR AQUI

                        cx3 = int(M3['m10']/M3['m00'])
                        cy3 = int(M3['m01']/M3['m00'])
                        cv2.circle(image, (cx3, cy3), 5, (255,0,0), -1)

                        cx7 = int(M7['m10']/M7['m00'])
                        cy7 = int(M7['m01']/M7['m00'])
                        cv2.circle(image, (cx7, cy7), 5, (255,0,0), -1)

                        cx8 = int(M8['m10']/M8['m00'])
                        cy8 = int(M8['m01']/M8['m00'])
                        cv2.circle(image, (cx8, cy8), 5, (255,0,0), -1)

                        
                        x1 = cy3
                        y1 = cx3
                        x2 = cy7
                        y2 = cx7
                        x3 = cy8
                        y3 = cx8
                        x300,x280,x320 = self.get_curva(x1,y1,x2,y2,x3,y3)
                        cv2.circle(image, (x300,300), 5, (0,255,0), -1)
                        cv2.circle(image, (x280,280), 5, (0,255,0), -1)
                        cv2.circle(image, (x320,320), 5, (0,255,0), -1)
                        self.erro = x300 - w/2 # Erro é o ponto do centróide em relação ao centro da câmera (w/2)
                        a = np.array([[x280,280],[x300,300],[x320,320]]) # Pontos para o cálculo da curvatura 

                elif M3['m00'] > 0 and M4['m00'] > 0 and M8['m00'] > 0 and flag == 0  : ## Caso não encontrou a linha no ponto central e encontrou pontos antes e depois da falha - ENTRAR AQUI

                        cx3 = int(M3['m10']/M3['m00'])
                        cy3 = int(M3['m01']/M3['m00'])
                        cv2.circle(image, (cx3, cy3), 5, (255,0,0), -1)

                        cx4 = int(M4['m10']/M4['m00'])
                        cy4 = int(M4['m01']/M4['m00'])
                        cv2.circle(image, (cx4, cy4), 5, (255,0,0), -1)

                        cx8 = int(M8['m10']/M8['m00'])
                        cy8 = int(M8['m01']/M8['m00'])
                        cv2.circle(image, (cx8, cy8), 5, (255,0,0), -1)

                        
                        x1 = cy3
                        y1 = cx3
                        x2 = cy4
                        y2 = cx4
                        x3 = cy8
                        y3 = cx8
                        x300,x280,x320 = self.get_curva(x1,y1,x2,y2,x3,y3)
                        cv2.circle(image, (x300,300), 5, (0,255,0), -1)
                        cv2.circle(image, (x280,280), 5, (0,255,0), -1)
                        cv2.circle(image, (x320,320), 5, (0,255,0), -1)
                        self.erro = x300 - w/2 # Erro é o ponto do centróide em relação ao centro da câmera (w/2)
                        a = np.array([[x280,280],[x300,300],[x320,320]]) # Pontos para o cálculo da curvatura 

                

                elif M3['m00'] > 0 and M4['m00'] > 0 and M5['m00'] > 0  : ## Caso não encontrou a linha no ponto central e encontrou 3 pontos depois da falha - ENTRAR AQUI

                        cx3 = int(M3['m10']/M3['m00'])
                        cy3 = int(M3['m01']/M3['m00'])
                        cv2.circle(image, (cx3, cy3), 5, (255,0,0), -1)

                        cx4 = int(M4['m10']/M4['m00'])
                        cy4 = int(M4['m01']/M4['m00'])
                        cv2.circle(image, (cx4, cy4), 5, (255,0,0), -1)

                        cx5 = int(M5['m10']/M5['m00'])
                        cy5 = int(M5['m01']/M5['m00'])
                        cv2.circle(image, (cx5, cy5), 5, (255,0,0), -1)

                        
                        x1 = cy3
                        y1 = cx3
                        x2 = cy4
                        y2 = cx4
                        x3 = cy5
                        y3 = cx5
                        x300,x280,x320 = self.get_curva(x1,y1,x2,y2,x3,y3)
                        cv2.circle(image, (x300,300), 5, (0,255,0), -1)
                        cv2.circle(image, (x280,280), 5, (0,255,0), -1)
                        cv2.circle(image, (x320,320), 5, (0,255,0), -1)
                        self.erro = x300 - w/2 # Erro é o ponto do centróide em relação ao centro da câmera (w/2)
                        a = np.array([[x280,280],[x300,300],[x320,320]]) # Pontos para o cálculo da curvatura 
                
                elif M6['m00'] > 0 and M7['m00'] > 0 and M8['m00'] > 0  : ## Caso não encontrou a linha no ponto central e encontrou 3 pontos antes da falha - ENTRAR AQUI

                        cx6 = int(M6['m10']/M6['m00'])
                        cy6 = int(M6['m01']/M6['m00'])
                        cv2.circle(image, (cx6, cy6), 5, (255,0,0), -1)

                        cx7 = int(M7['m10']/M7['m00'])
                        cy7 = int(M7['m01']/M7['m00'])
                        cv2.circle(image, (cx7, cy7), 5, (255,0,0), -1)

                        cx8 = int(M8['m10']/M8['m00'])
                        cy8 = int(M8['m01']/M8['m00'])
                        cv2.circle(image, (cx8, cy8), 5, (255,0,0), -1)

                        
                        x1 = cy6
                        y1 = cx6
                        x2 = cy7
                        y2 = cx7
                        x3 = cy8
                        y3 = cx8
                        x300,x280,x320 = self.get_curva(x1,y1,x2,y2,x3,y3)
                        cv2.circle(image, (x300,300), 5, (0,255,0), -1)
                        cv2.circle(image, (x280,280), 5, (0,255,0), -1)
                        cv2.circle(image, (x320,320), 5, (0,255,0), -1)
                        self.erro = x300 - w/2 # Erro é o ponto do centróide em relação ao centro da câmera (w/2)
                        a = np.array([[x280,280],[x300,300],[x320,320]]) # Pontos para o cálculo da curvatura 


                        


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
                vel_linearx = vel_linear
                vel_lineary = vel_linear
                conv = 0.00125*2 #Conversão de metros 
                altitude = 1.5
                ##h1 = altitudeZ  # 0.5 metros
                distH = altitudeZ*np.tan(np.pi/4)
                #print('disth',distH)
                self.ze = (self.erro)*(conv) # Z (metros) é igual ao erro (o quanto o centróide está distante do centro da câmera) multiplicado por um fator de conversão (pixel para metros)
                self.phir = np.arctan2(self.ze,distH) # ThetaR é igual a tangente entre o erro Z e a distância da câmera ao centróide (z) em Rad.

                self.r = self.r_anterior
                print('ze',self.ze)
                #print ('thetar',self.thetar)
                ## ('omega',self.omega)



                self.otimiza = self.callback_client(roll,pitch,yaw,self.ze,self.phir,self.r,distH,vel_linear) # Manda as variáveis de interesse para o otimizador.
                uoptimal = self.resposta.u ## A variavel de entrada u é recebida pela variável resposta.u .

                self.r_atual = (uoptimal*np.cos(self.phir) + curvature[1]*((np.cos(pitch)*np.cos(yaw)*vel_linearx)+(np.sin(roll)*np.sin(pitch)*np.cos(yaw)- np.cos(roll)*np.sin(yaw))*vel_lineary) / ((np.cos(self.phir) - curvature[1]*self.ze)*(np.cos(roll)/np.cos(pitch)))) # Cálculo do omega que será aplicado ao robô


                #if self.r_atual > 0.2 :
                #        self.r_atual = 0.2
                #if self.r_atual < -0.2 :
                #        self.r_atual = -0.2

                ## Verificar as variáveis uoptimal e curvature para verificar o que causa as oscilações quando existe falha na linha / A saturação neste valor ajudou na oscilação.




                self.twistS.twist.linear.x = vel_linear*np.cos(yaw)
                self.twistS.twist.linear.y = vel_linear*np.sin(yaw)
                #self.twistS.twist.linear.x = 0
                #self.twistS.twist.linear.y = 0
                #print(self.twistS.twist.linear.x)
                #print (self.r_atual)
                dt = rospy.get_time() - self.ptime
                self.twistS.twist.angular.z = self.pid(self.r_atual, 3,0.01,0,dt)
                #print (dt)
                self.r_anterior = self.r_atual   
                #print(self.ze)
     


                d_altitude = 1.5 - altitudeZ
                self.twistS.twist.linear.z = self.pidZ(d_altitude, 0.5, 0.0001, 0.1)
                self.cmd_vel_pub.publish(self.twistS)
                self.ptime = rospy.get_time()           
                
                ##self.cmd_vel_pub.publish(self.twist)
                #print (len(mask9[1]))
                cv2.imshow("window", image)
                #cv2.imshow("window1",mask9)
                #cv2.imshow("window1",mask1)

                cv2.waitKey(3)

rospy.init_node('line_follower')
follower = Follower()
rospy.spin()