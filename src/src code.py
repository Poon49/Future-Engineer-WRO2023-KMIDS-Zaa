import rclpy

from rclpy.node import Node
import time
from sensor_msgs.msg import Imu
import math
from sensor_msgs.msg import LaserScan
from gpiozero import LED
import RPi.GPIO as IO
import os, pigpio
import signal
import cv2
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from std_msgs.msg import Float32
import numpy as np

PWM_1 = 12
PWM_2 = 13
Servo_Pin = 23
regions_ = {
    'right': 0,
    'front': 0,
    'left': 0,
    'l': 0,
    'r': 0,
}

#Green
low_green=np.array([35,50,50])
high_green=np.array([85,255,255])

#Red
low_red=np.array([150,100,100])
high_red=np.array([180,255,255])

#kernel
kernel = np.ones((5,5),np.uint8)
kernel2 = np.ones((10,10),np.uint8)
current_frame = 0

class Future(Node):
    def __init__(self):
        super().__init__('Future')
        
        self.subImu = self.create_subscription(
            Float32,
            '/imu/data_raw',
            self.listener_imu,
            10)
        self.subLid = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_Lid,
            10)
        self.subcam = self.create_subscription(
        Image, 
        'video_frames', 
        self.listener_cam, 
        10)
        self.subcam # prevent unused variable warning
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        self.timer = self.create_timer(0.1, self.main_cb)
        self.subImu
        self.subLid
        self.yaw = 0
        self.laser = LaserScan()
        self.LastTimeImu = 0
        self.LastErrorImu = 0
        self.lastSteering = 0
        self.midServo = 1400
        self.maxServo = 1650
        self.minServo = 1100
        self.start_angle = 0
        self.init_angle = False
        self.pi = pigpio.pi()
        self.pi.set_mode(PWM_1, pigpio.OUTPUT)
        self.pi.set_mode(PWM_2, pigpio.OUTPUT)
        self.pi.set_mode(Servo_Pin, pigpio.OUTPUT)
        self.pi.set_PWM_frequency(Servo_Pin, 50)
        self.pi.set_PWM_frequency(PWM_1, 5000)
        self.pi.set_PWM_frequency(PWM_2, 5000)
        self.step = 1
        self.laps = 0
        self.angle = 0
        self.clockwise = False
        self.checkClockwise = True
        self.timetime = 0
        self.lap = 0
        self.rosF = 0.0
        self.rosR = 0.0
        self.rosL = 0.0
        self.keep = False
        self.j = 0
        self.l = 0
        self.countEnd = 0

        self.wait_cam_set = True
        self.wait_cam = False
        
        self.LastTimeImuD = 0
        self.LastErrorImuD = 0

        self.Forward = True
        self.LastStateForward = False
        self.running= True
        self.timeObs = 0
        self.awit = False
        self.wallDetect = True
        self.redObs = True
        self.get_logger().info("15")
        time.sleep(3)
        self.pi.set_PWM_dutycycle(PWM_1, 220)


    def main_cb(self): #Servo min 500 - 2500
        global current_frame
        global regions_
        global low_green
        global high_green
        global low_red
        global high_red
        global kernel
        global kernel2
        if(self.keep == False):
                    self.rosF = regions_['front']
                    self.rosL = regions_['left']
                    self.rosR = regions_['right']
                    if((self.rosF != 0 and self.rosL != 0 and self.rosR != 0)):
                        self.keep = True
        
        if self.wait_cam and self.keep:
            hhb = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

            green_mask = cv2.inRange(hhb, low_green, high_green)
            green_mask = cv2.erode(green_mask, kernel, iterations=1)
            green_mask = cv2.dilate(green_mask, kernel, iterations=4)
            green = cv2.bitwise_and(current_frame, current_frame, mask=green_mask)

            red_mask = cv2.inRange(hhb, low_red, high_red)
            red_mask = cv2.erode(red_mask, kernel, iterations=1)
            red_mask = cv2.dilate(red_mask, kernel, iterations=4)
            red = cv2.bitwise_and(current_frame, current_frame, mask=red_mask)

            # Find contours in the green and red masks
            green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Calculate the center of the green region
            # Calculate the center of the green region
            green_center_xx = []
            index_green = []
            red_center_xx = []
            area_green =[]
            area_red = []
            index_red = []
            green_center_yy = []
            red_center_yy = []
            green_line = 300
            red_line = 20
            k = 0
            if len(green_contours) > 0:
                for g in green_contours:
                    area = cv2.contourArea(g)
                    if area > 1000:
                        #green_contour = max(green_contours, key=cv2.contourArea)
                        M = cv2.moments(g)
                        if M["m00"] != 0:  # Ensure not dividing by zero
                            x = int(M["m10"] / M["m00"])
                            y = int(M["m01"] / M["m00"])
                            green_center_xx.append(x)
                            green_center_yy.append(y)
                            index_green.append(k)
                            area_green.append(area)
                            cv2.circle(green, (x, y), 5, (0, 255, 0), -1)  # Draw green dot
                '''        
                for i in range(len(green_contours)):
                    area = cv2.contourArea(green_contours[i])
                    if area > 1000:
                        hull = cv2.convexHull(green_contours[i])
                        x,y,w,h = cv2.boundingRect(hull)
                        
                        if(w>500):
                            cv2.drawContours(green, [hull], -1, (0, 0, 255), 2)
                        else:
                            cv2.drawContours(green, [hull], -1, (255, 0, 0), 2)
                '''    
                k+=1
            # Calculate the center of the red region
            k=0
            if len(red_contours) > 0:
                for r in red_contours:
                    area = cv2.contourArea(r)
                    if area > 1000:
                        #green_contour = max(green_contours, key=cv2.contourArea)
                        M = cv2.moments(r)
                        if M["m00"] != 0:  # Ensure not dividing by zero
                            x = int(M["m10"] / M["m00"])
                            y = int(M["m01"] / M["m00"])
                            red_center_xx.append(x)
                            red_center_yy.append(y)
                            index_red.append(k)
                            area_red.append(area)
                            cv2.circle(red, (x, y), 5, (0, 255, 0), -1)  # Draw green dot
                '''
                for i in range(len(red_contours)):
                    area = cv2.contourArea(red_contours[i])
                    if area > 1000:
                        hull = cv2.convexHull(red_contours[i])
                        x,y,w,h = cv2.boundingRect(hull)
                        
                        if(w>500):
                            cv2.drawContours(red, [hull], -1, (0, 0, 255), 2)
                        else:
                            cv2.drawContours(red, [hull], -1, (255, 0, 0), 2)
                '''
                k+=1

            countRed = len(index_red)
            countGreen = len(index_green)
            green_center_x = 160
            red_center_x = 160

            red_center_y = 0
            green_center_y = 0
            if countRed>0:
                ir = area_red.index(max(area_red))
                red_center_x = red_center_xx[ir]
                red_center_y = red_center_yy[ir]
                hull = cv2.convexHull(red_contours[index_red[ir]])
                cv2.drawContours(red, [hull], -1, (0, 0, 255), 2)
            if countGreen>0:
                ig = area_green.index(max(area_green))
                green_center_x = green_center_xx[ig]
                green_center_y = green_center_xx[ig]
                hull = cv2.convexHull(green_contours[index_green[ig]])
                cv2.drawContours(green, [hull], -1, (0, 0, 255), 2)
            if countGreen or countRed:
                self.get_logger().info("Red = %d  green = %d"%(countRed ,countGreen))
            cv2.line(current_frame, (green_line,0), (green_line, 240), (0,10,255),1)
            cv2.line(current_frame, (red_line,0), (red_line, 240), (0,240,10),1)
            
            # Display image
            '''
            ox = []
            ox = self.findX()
            if len(ox):
                for i in ox:
                    cv2.circle(current_frame, (i, 100), 2, (0, 255, 0), -1)
            '''
            

            
            self.wallDetect = True
            
            if (self.step != 2 ):
                angleDetect = False
                if abs(self.FindErrorHeading(self.angle))<40:
                    angleDetect = True
                else:
                    angleDetect = False
                if(countRed >  0 and countGreen == 0):
                    y = self.findRangeObscure(red_center_y)
                    self.get_logger().info(str(y))
                    if ( red_center_x > red_line  and y<0.6 and  red_center_x < 320 and  angleDetect):
                        self.prints("loop 1")
                        cv2.rectangle(current_frame, (red_center_x - 10, red_center_y - 10), (red_center_x + 10, red_center_y + 10), (0,240,10),1) 
                        self.dodge(red_center_x,y,'Red')
                        self.timeObs = time.time()
                        self.running = False
                        self.wallDetect = False
                        self.redObs = True
                elif(countGreen >0 and countRed == 0 and angleDetect):
                    y = self.findRangeObscure(green_center_y)
                    self.prints(str(y))
                    if(green_center_x > 0 and  green_center_x < green_line and (self.FindErrorHeading(self.angle))<40 and y<0.6):
                        self.prints("loop 2")
                        cv2.rectangle(current_frame, (green_center_x - 10, green_center_y - 10), (green_center_x + 10, green_center_y + 10), (0,240,10),1) 
                        self.dodge(green_center_x,y,'Green')
                        self.timeObs = time.time()
                        self.running = False   
                        self.wallDetect = False  
                        self.redObs = False               
                elif(countGreen > 0 and countRed > 0 and angleDetect):
                    if(red_center_y > green_center_y):
                        y = self.findRangeObscure(red_center_y)
                        if(red_center_x > red_line  and red_center_x < 320  and y<0.6 and self.FindErrorHeading(self.angle)<40):
                            self.prints("loop -1")
                            cv2.rectangle(current_frame, (red_center_x - 10, red_center_y - 10), (red_center_x + 10, red_center_y + 10), (0,240,10),1) 
                            self.dodge(red_center_x,y,'Red')
                            self.timeObs = time.time()
                            self.running = False
                            self.wallDetect = False
                            self.redObs = True
                            
                    elif(green_center_y > red_center_y):
                        y = self.findRangeObscure(green_center_x)
                        if (green_center_x > 0 and  green_center_x < green_line and y<0.6 and self.FindErrorHeading(self.angle)>-40):
                            self.prints("loop 3-2")
                            cv2.rectangle(current_frame, (green_center_x - 10, green_center_y - 10), (green_center_x + 10, green_center_y + 10), (0,240,10),1) 
                            self.dodge(green_center_x,y,'Green')
                            self.timeObs = time.time()
                            self.running = False
                            self.wallDetect = False
                            self.redObs = False
                
                img = cv2.bitwise_or(green,red,mask = None)
                cv2.imshow("img", cv2.hconcat([current_frame,img]) )
                cv2.waitKey(1)
                            #self.findRangeObscure(red_center_x)<0.6
                
                if self.wallDetect:
                    #self.get_logger().info("wall Detect")
                    t = time.time()-self.timeObs
                    self.get_logger().info("wall detect time %f  angle %d regions %f  angle %f"%(t,self.angle,regions_['l'],self.angle))
                    rangeObs = 0.0
                    angle = self.angle
                    if self.redObs:
                        rangeObs = regions_['l']
                        angle+=20
                        if angle>359:
                            angle-=359
                        elif angle<0:
                            angle+=359        
                    else:
                        rangeObs = regions_['r']
                        angle-=20
                        if angle>359:
                            angle-=359
                        elif angle<0:
                            angle+=359  
                    if (t<0.1):
                        pass
                    elif(rangeObs<0.4):
                        self.followIMU(angle,5)
                        self.prints("W1")
                    elif(t<1):
                        self.prints("W2")
                        yy = self.FindErrorHeading(self.angle)
                        if yy < 0:
                            self.followIMU(yy+40,20)
                        else:
                            self.followIMU(yy-40,20)
                    else:
                            self.prints("W3")
                            LR, WALL = self.follow_wall()
                            self.get_logger().info("LR %f, WALL %f"%(LR,WALL))
                            if LR < 0 and LR>-35:#Left Wall Follow
                                if  WALL > 0.6 : 
                                    self.pi.set_servo_pulsewidth(Servo_Pin, 1600)
                                    self.running = False
                                elif  WALL > 0.02 and WALL < 0.4: 
                                    self.pi.set_servo_pulsewidth(Servo_Pin, 1200)
                                    self.running = False
                                else:
                                    self.running = True
                            elif LR > 0 and LR < 35:#Right Wall Follow
                                if  WALL > 0.6 : 
                                    self.pi.set_servo_pulsewidth(Servo_Pin, 1200)
                                    self.running = False
                                elif  WALL > 0.02 and WALL < 0.4: 
                                    self.pi.set_servo_pulsewidth(Servo_Pin, 1600)
                                    self.running = False
                                else:
                                    self.running = True
                            else:
                                self.running = True
            '''
            if self.LastStateForward != self.Forward :
                self.LastStateForward = self.Forward
                self.pi.set_PWM_dutycycle(PWM_2, 0)
                self.pi.set_PWM_dutycycle(PWM_1, 0)
                time.sleep(1)

            if self.Forward:
                self.pi.set_PWM_dutycycle(PWM_1, 0)
                self.pi.set_PWM_dutycycle(PWM_2, 0)
            else:
                self.pi.set_PWM_dutycycle(PWM_2, 0)
                self.pi.set_PWM_dutycycle(PWM_1, 0)
        
            '''
            
            if(self.lap < 12 ):
                if(regions_['front'] > 0 ):
                    if(regions_['front'] < 0.7 and abs(self.FindErrorHeading(self.angle))<25.0 and time.time()-self.timetime>3 ):
                        if(self.step == 1):
                            if(regions_['right'] > 1.2):
                                self.step = 2
                                if(self.checkClockwise == True):
                                    self.clockwise = False
                                    self.checkClockwise = False
                                if  (self.clockwise == False):
                                    self.angle = self.angle + 90
                                    if(self.angle<0):
                                        self.angle = 270
                                    if(self.angle>359):
                                        self.angle = 0


                            elif(regions_['left'] > 1.2):
                                self.step = 2
                                if(self.checkClockwise):
                                    self.clockwise = True
                                    self.checkClockwise = False
                                if(self.clockwise == True):
                                    self.angle = self.angle - 90
                                    if(self.angle>359):
                                        self.angle = 0
                                    if(self.angle<0):
                                        self.angle = 270
            
            else:
                if (time.time()-self.timetime > 2):
                    self.pi.set_PWM_dutycycle(PWM_1, 0)
                    self.pi.set_PWM_dutycycle(PWM_2, 0)
                    time.sleep(1000)
                else:
                   self.followIMU(self.angle, 15)
            
            if(self.step == 2):
                #self.pi.set_PWM_dutycycle(PWM_1, 254)
                self.followIMU(self.angle, 15)
                if(abs(self.yaw-self.angle)<30.0):
                    self.step = 1
                    self.lap += 1
                    self.timetime = time.time()
            
            if(self.running):
                self.get_logger().info("running")
                if(self.step == 1):
                    self.followIMU(self.angle, 15)
                elif(self.step == 3):
                    self.followIMU(self.angle, 15)
                    if(time.time()-self.timetime > 10):
                        self.step = 1
            
            '''
                    elif(self.lap == 11):
                        p = 0
                        if(self.clockwise == True):
                            p = self.rosR + 0.2
                        else:
                            p = self.rosL + 0.2
                        if(regions_['front'] > 0 ):
                            if(regions_['front'] < p and abs(self.FindErrorHeading(self.angle)<15.0 and time.time()-self.timetime>3)):
                                if(self.step == 1):
                                    if(regions_['right'] > 0.9):
                                        self.step = 2
                                        if(self.checkClockwise == True):
                                            self.clockwise = False
                                            self.checkClockwise = False
                                        if  (self.clockwise == False):
                                            self.angle = self.angle + 90
                                            if(self.angle<0):
                                                self.angle = 270
                                            if(self.angle>359):
                                                self.angle = 0


                                    elif(regions_['left'] > 0.9):
                                        self.step = 2
                                        if(self.checkClockwise):
                                            self.clockwise = True
                                            self.checkClockwise = False
                                        if(self.clockwise == True):
                                            self.angle = self.angle - 90
                                            if(self.angle>359):
                                                self.angle = 0
                                            if(self.angle<0):
                                                self.angle = 270
                    
                        
                            if(self.step == 1):
                                #self.pi.set_PWM_dutycycle(PWM_1, 230)
                                self.followIMU(self.angle, 15)


                            elif(self.step == 2):
                                #self.pi.set_PWM_dutycycle(PWM_1, 200)
                                self.followIMU(self.angle, 15)
                                if(abs(self.yaw-self.angle)<20.0):
                                    self.step = 1
                                    self.lap += 1
                                    self.timetime = time.time()
                        
                            elif(self.step == 3):
                                self.pi.set_PWM_dutycycle(PWM_1, 0)
                                self.followIMU(self.angle, 15)
                                if(time.time()-self.timetime > 10):
                                    self.step = 1

                    else:
                        if(regions_['front'] < self.rosF and (self.yaw < 10 or self.yaw > 350) and time.time() - self.timetime > 1):
                            self.countEnd += 1    
                        else:
                            #self.pi.set_PWM_dutycycle(PWM_1, 220)
                            self.followIMU(0, 20)
                    if(self.countEnd > 3):
                        self.pi.set_servo_pulsewidth(Servo_Pin, self.midServo)
                        self.pi.set_PWM_dutycycle(PWM_1, 0)
                        time.sleep(1000)  
                '''   

    def prints(self, st):
        self.get_logger().info(st)       
    def findX(self):
        k=0
        obx = []
        for i in self.laser.ranges:
            if k>0 and k<225:
                if(i>0 and i<0.2):
                    #self.get_logger().info("laser= %f  index= %d" %(i, k))
                    obx.append(int((-3.2*k+544)))
            k+=1
        return obx
    
    def follow_wall(self):
        x = self.FindErrorHeading(self.angle)
        lidar_offset = 0
        if x > 0: #turn left
            lidar_offset = 225-int(x/0.8)
        elif x < 0: #right
            lidar_offset = 0-int(x/0.8)

        if lidar_offset > 225:
            lidar_offset = 225
        elif lidar_offset<12:
            lidar_offset = 12

        try:
            return x,min(min(self.laser.ranges[lidar_offset-12:lidar_offset+12]),10)
        except:
            return x,10

    def findRangeObscure(self, x):
        y = int(-0.3125*(x) +180)
        #self.get_logger().info(str(x) + '  ' + str(y))
        if y < 25:
            y = 25
        elif y> 235:
            y = 235
        
        try:
            return min(min(self.laser.ranges[y-25:y+25]),10)
        except:
            return 10
    def dodge(self,x, y, gr):
        #self.pi.set_PWM_dutycycle(PWM_1, self.sp_step1)
        kp = 10
        x = (0.8-y)*100.0+self.angle
        self.get_logger().info("dodge : "+str(x))
        if(x > 45):
            x = 45
        elif(x < 20):
            x = 20
        kd = 0.6
        '''
        if (gr == 'Green'):
            error = 220 - x
        elif (gr == 'Red'):
            error = x - 100
        '''
        if gr=='Green':
            x = -1*x
        self.followIMU(x,15)
        '''
        CurrentTime = (time.time()*1000)
        rateError = (error-self.LastErrorImuD)/(CurrentTime - self.LastTimeImuD) 
        self.LastErrorImuD = error
        self.LastTimeImuD = CurrentTime
        PD = int((kp*error) + (kd*rateError))
        if (gr == 'Green'):
            pulseServo = self.midServo - PD
        elif(gr == 'Red'):
            pulseServo = self.midServo + PD
        if(pulseServo>self.maxServo):
            pulseServo = self.maxServo
        if(pulseServo< self.minServo):
            pulseServo = self.minServo
        #lastSteering = pulseServo
        
        if (gr == 'Green'):
            wall = regions_['left']
        elif (gr == 'Red'):
            wall = regions_['right']
        if (wall > 0.08):
            self.pi.set_servo_pulsewidth(Servo_Pin, pulseServo)
            self.Forward = True
        else:
            self.Forward = False
            self.followIMU(self.angle,15)
        '''

    def follow_imu(self, target):
        offset = target - int(self.yaw)
        #self.get_logger().info('before: "%f"' % offset)
        if(offset < -180):
            offset = offset + 360
        elif(offset > 180):
            offset = offset - 360
        #self.get_logger().info('after: "%f"' % offset)
        if(offset > 0):
            self.pi.set_servo_pulsewidth(Servo_Pin, 1650)
        elif(offset < 0):
            self.pi.set_servo_pulsewidth(Servo_Pin, 1100)
        else:
            self.pi.set_servo_pulsewidth(Servo_Pin, 1400)
  
    def FindErrorHeading(self, target):
        offset = target - int(self.yaw)
        #self.get_logger().info('before: "%f"' % offset)
        if(offset < -180):
            offset = offset + 359
        elif(offset > 180):
            offset = offset - 359
        return offset



    def listener_imu(self,data):
        #pitch,roll, self.yaw = self.euler_from_quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        #self.yaw = self.yaw * 180.0/3.14159265
        #self.get_logger().info('Yaw: "%f"' % self.yaw)
        self.yaw = data.data
        if(self.init_angle == False):
            self.start_angle = self.yaw
            self.init_angle = True
        self.yaw = self.yaw - self.start_angle
        if(self.yaw < 0):
            self.yaw = self.yaw + 359
        elif(self.yaw > 359):
            self.yaw = self.yaw - 359
        
        #self.get_logger().info("%f %f %f" %(pitch, roll, self.yaw))

    def listener_cam(self, data):
        # Callback function.
        # Convert ROS Image message to OpenCV image
        global low_green, high_green, low_red, high_red, kernel, current_frame
        if self.wait_cam_set:
            self.wait_cam = True
            self.wait_cam_set = False
        current_frame = self.br.imgmsg_to_cv2(data)

    
    def listener_Lid(self, msg):

        global regions_
        self.laser = msg
        shift = 0
        x = self.FindErrorHeading(self.angle)
        lidar_offset = 112-int(x/0.8)
        if lidar_offset >210:
            lidar_offset = 210
        elif lidar_offset < 15:
            lidar_offset = 15
        regions_ = {
            #'left':  min(min(msg.ranges[188+shift:225+shift]), 10),
            'left':  min(min(msg.ranges[205+shift:225+shift]), 10),
            'front':  min(min(msg.ranges[lidar_offset-10+shift:lidar_offset+10+shift]), 10),
            'right':   min(min(msg.ranges[0+shift:20+shift]), 10),
            'l':  min(min(msg.ranges[lidar_offset+shift:225+shift]), 10),
            'r':min(min(msg.ranges[0+shift:lidar_offset+shift]), 10),
        }
        #for i in range(359):

    def euler_from_quaternion(self,x, y, z, w):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

     
        return roll, pitch, yaw # in radians
    
    def findErrorImu(self,heading):
        error = heading - self.yaw
        if(error > 180):
            error =  error-360
        elif(error < -180):
            error =  360+error
        return error



    def followIMU(self,heading,kp):
        kd = 0.2
        error = heading-int(self.yaw)
        if(error > 180):
            error =  error-359
        elif(error < -180):
            error =  359+error

        CurrentTime = (time.time()*1000)
        rateError = (error-self.LastErrorImu)/(CurrentTime - self.LastTimeImu) 
        self.LastErrorImu = error
        self.LastTimeImu = CurrentTime
        PD = int((kp*error) + (kd*rateError))
        pulseServo = self.midServo + PD
        if(pulseServo>self.maxServo):
            pulseServo = self.maxServo
        if(pulseServo< self.minServo):
            pulseServo = self.minServo
        #lastSteering = pulseServo
        self.pi.set_servo_pulsewidth(Servo_Pin, pulseServo)
        #self.get_logger().info("%d %f" %(error, pulseServo))
    
    def stopMotor(self):
        self.pi.set_PWM_dutycycle(PWM_2, 0)
        self.pi.set_PWM_dutycycle(PWM_1, 0)
        self.pi.set_PWM_frequency(PWM_1, 0)
        self.pi.set_PWM_frequency(PWM_2, 0)
        #self.get_logger().info("shutdown")

    def listener_callback(self, data):
        # Callback function.
        # Convert ROS Image message to OpenCV image
        global low_green, high_green, low_red, high_red, kernel

        current_frame = self.br.imgmsg_to_cv2(data)
        hhb = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        green_mask = cv2.inRange(hhb, low_green, high_green)
        green_mask = cv2.erode(green_mask, kernel, iterations=1)
        green = cv2.bitwise_and(current_frame, current_frame, mask=green_mask)

        red_mask = cv2.inRange(hhb, low_red, high_red)
        red_mask = cv2.erode(red_mask, kernel, iterations=1)
        red = cv2.bitwise_and(current_frame, current_frame, mask=red_mask)

        # Find contours in the green and red masks
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Calculate the center of the green region
        # Calculate the center of the green region
        if len(green_contours) > 0:
            green_contour = max(green_contours, key=cv2.contourArea)
            M = cv2.moments(green_contour)
            if M["m00"] != 0:  # Ensure not dividing by zero
                green_center_x = int(M["m10"] / M["m00"])
                green_center_y = int(M["m01"] / M["m00"])
                cv2.circle(green, (green_center_x, green_center_y), 5, (0, 255, 0), -1)  # Draw green dot

        # Calculate the center of the red region
        if len(red_contours) > 0:
            red_contour = max(red_contours, key=cv2.contourArea)
            M = cv2.moments(red_contour)
            if M["m00"] != 0:  # Ensure not dividing by zero
                red_center_x = int(M["m10"] / M["m00"])
                red_center_y = int(M["m01"] / M["m00"])
                cv2.circle(red, (red_center_x, red_center_y), 5, (0, 0, 255), -1)  # Draw red dot


        # Display image
        cv2.imshow("green", green)
        cv2.imshow("red", red)

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    all_programs = Future()
    rclpy.spin(all_programs)
    all_programs.stopMotor()
    all_programs.destroy_node()
    rclpy.shutdown()


 
def handler(signum, frame):
    msg = "Ctrl-c was pressed."
    pi = pigpio.pi()
    pi.set_mode(PWM_1, pigpio.OUTPUT)
    pi.set_mode(PWM_2, pigpio.OUTPUT)
    pi.set_mode(Servo_Pin, pigpio.OUTPUT)
    pi.set_PWM_frequency(Servo_Pin, 50)
    pi.set_servo_pulsewidth(Servo_Pin, 1400)
    pi.set_PWM_frequency(Servo_Pin, 0)
    pi.set_PWM_dutycycle(PWM_1, 0)
    pi.set_PWM_dutycycle(PWM_2, 0)
    pi.set_PWM_frequency(PWM_1, 0)
    pi.set_PWM_frequency(PWM_2, 0)

signal.signal(signal.SIGINT, handler)

if __name__ == '__main__':
    main()
