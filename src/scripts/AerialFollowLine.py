#!/usr/bin/env python



import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
import numpy as np
from nav_msgs.msg import Odometry


#variaveis globais
theta_r = 0
altitudeZ = 0
velAnteriorX = 0
velAnteriorY = 0
error = 0
d_linha = 0
ang = 0

integral = 0.0
last_erro = 0.0
integralZ = 0.0
last_erroZ = 0.0

class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()
                #cv2.namedWindow("window", 1)

                #self.image_sub = rospy.Subscriber("/gazebo/camera/image_raw",
                #        Image, self.image_callback)
                print 'passou 1\n'
                self.image_sub = rospy.Subscriber("/gazebo/camera/image_raw",
                        Image, self.image_callback2)
                
                self.odom_sub = rospy.Subscriber("/mavros/local_position/odom",
                        Odometry, self.Odom_Callback)
                print 'passou 2\n'
                self.cmd_vel_pub = rospy.Publisher('/setpoint_offboard_TwstS',TwistStamped,queue_size=1)

                self.twistS = TwistStamped()
                self.odom = Odometry()

        '''def image_callback(self, msg):

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                lower_yellow = numpy.array([ 0, 0, 0])
                upper_yellow = numpy.array([15, 15, 15])
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

                h, w, d = image.shape
                search_top = 3*h/4
                search_bot = search_top + 200
                mask[0:search_top, 0:w] = 0
                mask[search_bot:h, 0:w] = 0


                M = cv2.moments(mask)
                if M['m00'] > 0:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
#The proportional controller is implemented in the following four lines which
#is reposible of linear scaling of an error to drive the control output.
                        err = cx - w/2
                        #print('err',err)
                        #print('cx',cx)
                        z = -float(err) / 1000
                        print('z',z)
                        
                        self.twist.linear.x = 0.4
                        self.twist.angular.z = float(err) / 1000
                        #self.twist.angular.z = 0
                        self.cmd_vel_pub.publish(self.twist)
                cv2.imshow("window", image)
                cv2.imshow("window1",mask )

                cv2.waitKey(3)'''

        def pidControl(self, erro, kp, ki, kd):
            global integral, last_erro
            integral += erro
            derivative = erro - last_erro
            last_erro = erro
            return kp * erro + ki * integral + kd * derivative

        def pidZ(self, erro, kp, ki, kd):
            global integralZ, last_erroZ
            integralZ += erro
            derivative = erro - last_erroZ
            last_erroZ = erro
            return kp * erro + ki * integralZ + kd * derivative

        def image_callback2(self, msg):
            global theta_r
            global altitudeZ
            global velAnteriorX
            global velAnteriorY
            global error
            global d_linha
            global ang
            V = 0.2
            k_ang = 0.09
            k_lin = -0.0000
            #print "In the function\n"
            #image = frame.array
            image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            '''hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            #Blackline = cv2.inRange(image, (0,0,200), (30,30,255))
            Blackline = cv2.inRange(hsv, (0,0,200), (30,30,255))
            print image #+ "\n"
            kernel = np.ones((3,3), np.uint8)
            Blackline = cv2.erode(Blackline, kernel, iterations=5)
            Blackline = cv2.dilate(Blackline, kernel, iterations=9)	'''
            imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            ret, thresh = cv2.threshold(imgray, 130, 255, 0)
            img_blk,contours_blk, hierarchy_blk = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            #contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

            if len(contours_blk) > 0:
                blackbox = cv2.minAreaRect(contours_blk[0])
                (x_min, y_min), (w_min, h_min), ang = blackbox
                if ang < -45 :
                    ang = 90 + ang
                if w_min < h_min and ang > 0:
                    ang = (90-ang)*-1
                if w_min > h_min and ang < 0:
                    ang = 90 + ang	  
                setpoint = 320
                error = int(x_min - setpoint) 
                ang = int(ang)
                
                box = cv2.boxPoints(blackbox)
                box = np.int0(box)
                cv2.drawContours(image,[box],0,(0,0,255),3)	 
                cv2.putText(image,str(ang),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(image,str(error),(10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                cv2.line(image, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)
                #cv2.imshow("orginal with line", image)
                #cv2.imshow("Image", image)
                #cv2.imshow("Mask", Blackline)
                #cv2.waitKey(3)

                #CONTROLADOR
                angulo_linha = theta_r+(ang*3.14159/180)
                d_linha = error + 160
                #d_linha = 0
                '''
                e0 = 160
                if(abs(d_linha)<e0):
                    self.twistS.twist.linear.x = V*np.cos(angulo_linha)
                    self.twistS.twist.linear.y = V*np.sin(angulo_linha)
                    #variavel = 1
                else:
                    self.twistS.twist.linear.x = d_linha*k_lin*np.sin(angulo_linha)
                    self.twistS.twist.linear.y = d_linha*k_lin*np.cos(angulo_linha) '''
                
                #self.twistS.twist.angular.z = -self.pidControl(ang, k_ang, 0.0, 0.0)
                #self.twistS.twist.angular.z = ang*k_ang
                #self.twistS.twist.linear.z = k_lin*d_altitude
                
                
                #self.twistS.twist.linear.x = 10
                #print "Error: ", error+160, "Angle: ",ang, "w: ",self.twistS.twist.angular.z, 'Angulo linha: ', angulo_linha, "\n"

            d_altitude = 1.3 - altitudeZ
            #print "Altitude: ", altitudeZ, "Ang:", ang
            #self.twistS.twist.linear.x = V*np.cos(theta_r) / (abs(d_linha) + 1.0) + d_linha*k_lin*np.sin(theta_r)
            #self.twistS.twist.linear.y = V*np.sin(theta_r) / (abs(d_linha) + 1.0) + d_linha*k_lin*np.cos(theta_r)

            self.twistS.twist.linear.x = V*np.cos(theta_r) + d_linha*k_lin*np.sin(-theta_r)
            self.twistS.twist.linear.y = V*np.sin(theta_r) + d_linha*k_lin*np.cos(-theta_r)

            self.twistS.twist.angular.z = -self.pidControl(ang, k_ang, 0.0, 0.0)

            self.twistS.twist.linear.z = self.pidZ(d_altitude, 0.5, 0.0001, 0.1)
            self.cmd_vel_pub.publish(self.twistS)
            
                
            
        
        def Odom_Callback(self, msg):
            self.odom = msg

            #print "Vx: ", self.odom.twist.twist.linear.x, '\n'

            global theta_r
            global altitudeZ
            global velAnteriorX
            global velAnteriorY

            pi = 3.1415926535897932384642433832795

            altitudeZ = self.odom.pose.pose.position.z
            theta_r = np.arctan2(2*self.odom.pose.pose.orientation.w*self.odom.pose.pose.orientation.z,1-2*self.odom.pose.pose.orientation.z*self.odom.pose.pose.orientation.z)
            #theta_r += pi
            #theta_r = 2*pi - theta_r
            velAnteriorX = self.odom.twist.twist.linear.x
            velAnteriorY = self.odom.twist.twist.linear.y



rospy.init_node('line_follower')
#print "Algum coisa"
follower = Follower()
rospy.spin()
