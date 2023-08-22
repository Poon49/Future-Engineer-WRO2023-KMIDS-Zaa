# WRO-Future-Engineers 2023 (KMIDS Zaa Team)
This repository contains codes and working principle that are used to creating the autonomous vehicle for competition.

### Car electromechanical parts:
Our car contains the follow parts:
1.	Raspberry Pi 4 – The brain of the robot, use to receive the frames from the camera, give an order to the medium motor and servo motor, and use for keeping the value from the Gyro sensor (BNO055).
   
![image](https://github.com/Poon49/Future-Engineer-WRO2023-KMIDS-Zaa/assets/76239146/99f7a1bc-3fa4-47f0-917c-8d65e9b11c63)


Specification 
 - Broadcom BCM2711, Quad core Cortex-A72 (ARM v8) 64-bit SoC @ 1.8GHz
   1GB, 2GB, 4GB or 8GB LPDDR4-3200 SDRAM (depending on model)
 - 2.4 GHz and 5.0 GHz IEEE 802.11ac wireless, Bluetooth 5.0, BLE
 - Gigabit Ethernet
 - 2 USB 3.0 ports; 2 USB 2.0 ports.
 - Raspberry Pi standard 40 pin GPIO header (fully backwards compatible with previous boards)
 - 2 × micro-HDMI® ports (up to 4kp60 supported)
 - 2-lane MIPI DSI display port
 - 2-lane MIPI CSI camera port
 - 4-pole stereo audio and composite video port
 - H.265 (4kp60 decode), H264 (1080p60 decode, 1080p30 encode)
 - OpenGL ES 3.1, Vulkan 1.0
 - Micro-SD card slot for loading operating system and data storage
 - 5V DC via USB-C connector (minimum 3A*)
 - 5V DC via GPIO header (minimum 3A*)
 - Power over Ethernet (PoE) enabled (requires separate PoE HAT)
 - Operating temperature: 0 – 50 degrees C ambient
 https://www.raspberrypi.com/products/raspberry-pi-4-model-b/specifications/
2.	Differential Lego- Gear
   
      ![image](https://github.com/Poon49/Future-Engineer-WRO2023-KMIDS-Zaa/assets/76239146/f08a427d-a039-49e9-9104-1b26b1bc8983)


                                                
3.	EV3 medium motor 
The Medium Motor also includes a built-in Rotation Sensor (with 1-degree resolution), but it is smaller and lighter than the Large Motor. That means it is able to respond more quickly than the Large Motor. The Medium Motor can be programmed to turn on or off, control its power level, or to run for a specified amount of time or rotations.

     ![image](https://github.com/Poon49/Future-Engineer-WRO2023-KMIDS-Zaa/assets/76239146/67e4baf2-e0e3-4a88-986c-2da8c2f5b72e)


   - Speed 240-250 rpm
   - Dynamic Torque  8 N/cm 
   - Torque 12 N/cm                               

4.	Servo motor MG90S : MG90S servo, Metal gear with one bearing can rotate approximately
   180 degrees (90 in each direction),

    ![image](https://github.com/Poon49/Future-Engineer-WRO2023-KMIDS-Zaa/assets/76239146/3a375326-7920-44db-8653-2d28afc5a695)

Specifications 
- Weight: 13.4 g
-  Dimension: 22.5 x 12 x 35.5
-  Stall torque: 1.8 kgf·cm (4.8V )
-  Operating speed: 0.1 s/60 degree
-  Operating voltage: 4.8 V – 6V
-  Dead band width: 5 µs

5. ESP32-DevKitC-WROOM-32U module

![image](https://github.com/Poon49/Future-Engineer-WRO2023-KMIDS-Zaa/assets/76239146/61fbd9cf-e68c-4662-a396-899e0dbe7449)



- Product Category:	WiFi Development Tools - 802.11
- RoHS:	N
- Protocol Supported:	802.11 b/g/n
- Frequency:	2.4 GHz to 2.5 GHz
- Operating Supply Voltage:	5 V
- Protocol - WiFi - 802.11:	WiFi
- Dimensions:	54.4 mm x 27.9 mm
- Interface Type:	USB

6.	 BNO055 (Gyro sensor) :
   
       ![image](https://github.com/Poon49/Future-Engineer-WRO2023-KMIDS-Zaa/assets/76239146/3b3a0e9b-205f-4812-9b9e-b1fa66ef5c1a)

                
The BNO055 can output the following sensor data:
  - Absolute Orientation (Euler Vector, 100Hz) Three axis orientation data based on a 360° sphere
  - Absolute Orientation (Quaternion, 100Hz) Four-point quaternion output for more accurate data 
    manipulation
  - Angular Velocity Vector (100Hz) Three axis of 'rotation speed' in rad/s
  - Acceleration Vector (100Hz) Three axis of acceleration (gravity + linear motion) in m/s^2
  - Magnetic Field Strength Vector (20Hz) Three axis of magnetic field sensing in micro-Tesla (uT)
  - Linear Acceleration Vector (100Hz) Three axis of linear acceleration data (acceleration minus 
    gravity) in m/s^2
  - Gravity Vector (100Hz) Three axis of gravitational acceleration (minus any movement) in m/s^2
  - Temperature (1Hz) Ambient temperature in degrees Celsius
    
Datasheet :
https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf

Library :
https://github.com/adafruit/Adafruit_BNO055

7.	Step-Down Module 5.0 HW-688,24V/12V To 5V USB DC-DC
   
      ![image](https://github.com/Poon49/Future-Engineer-WRO2023-KMIDS-Zaa/assets/76239146/c1147db0-2194-446d-b596-359ff14be7e9)

                                      
Specifications:
  - Input: DC 9V--36V
  - Output: 5.2V/5A/25W
  - Material: electronic components
  - Size: 65*28*13mm
                               
8.	DTOF Lidar D300 :

  	 ![image](https://github.com/Poon49/Future-Engineer-WRO2023-KMIDS-Zaa/assets/76239146/120ea4f2-add5-40c5-a7f0-19abf95b6133)  ![image](https://github.com/Poon49/Future-Engineer-WRO2023-KMIDS-Zaa/assets/76239146/707dbe24-6391-4d58-b359-3ea840bf13a6)
                                

Product parameters
  - Model    :   LD19/D300
  - Type            :     Close range
  - Ranging principle            :     TOF ranging supports indoor and outdoor environment
  - Scan angle                       :     360°
  - Angle resolution              :     0.8°
  - Measurement frequency  :     4500/s
  - Scan frequency                :     10Hz
  - Shape Size                       :     38.59*38.59*34.8mm
  - Weight                             :     42g
  - Measuring distance accuracy      :     Ranging 3m-12m: +20mm
  - Light source                                :     905nm Laser
  - Measuring radius                        :      White object: 13.4m Black objects: 13.4m
  - Minimum measuring distance    :     0.02m
  - Anti-ambient light intensity       :     30KLux (Unit of light intensity)
  - Power supply                             :     5VDC
  - Temperature                               :     Working: -10 -40°Storage: -30°-70°
  - Drive mode                                :     Built-in brushless motor
  - Windows support                       :     Provide Windows PC software
  - ROS support                                                     :     ROS1/ROS2
  - Standard asynchronous serial port (UART)     :     Communication interface


9.	Sony IMX219 (8 MP )camera module with M12 lens LS40136 for Raspberry Pi:
 
   ![image](https://github.com/Poon49/Future-Engineer-WRO2023-KMIDS-Zaa/assets/76239146/ba12e392-e3a4-4454-af30-5a9c78d2b9d9)

Specifications
Camera :
  - Image Sensor	IMX219
  - Still Resolution	8 Megapixels
  - Video Modes	1080p47, 1640 × 1232p41 and 640 × 480p206
  - Sensor Resolution	3280 × 2464 pixels
  - Sensor image area	3.68 x 2.76 mm (4.6 mm diagonal)
  - Pixel Size	1.12 µm x 1.12 µm
  - Optical Size	1/4"
IR Sensitivity	Integral IR-cut Filter, visible light only
Lens :
  - Focus Type	Manual Focus
  - Focal Length	2.8mm
  - F.NO	F2.2
  - Field of View(FOV)	70°(H)
  - Lens Mount	M12 Mount
    Electrical & Mechanical
Camera Board Size 	36mm x 36mm
Peaking Current	300mA

10.	STCS34725 RGB Color Sensor Module:

      ![image](https://github.com/Poon49/Future-Engineer-WRO2023-KMIDS-Zaa/assets/76239146/eee7470c-9a9c-4193-bee6-6dca20c11d28)

   White Light Sensing with IR Blocking Filter. Programmable analog gain and integration time. Programmable upper and lower thresholds with persistence filter. 65-uA Wait State with Programmable Wait State Time from 2.4 ms to &gt; 7 Seconds. I2C Fast Mode Compatible Interface. Input Voltage Levels Compatible with VDD or 1.8 V Bus. Register Set and Pin Compatible with the TCS3x71 Series. Small 2 mm*2.4 mm Dual Flat No-Lead (FN).
  - Full RGB Color sensor.
  - Communication via I2C.
  - Operating voltage: 3.0V to 5.0 VDC

## Calculation speed and time

 ![image](https://github.com/Poon49/Future-Engineer-WRO2023-KMIDS-Zaa/assets/76239146/e92266c6-9c40-4cdc-b07d-9d4ee45fac76)



## Modules used:
Other modules include:
1.	Board- Raspberry Pi 4 4GB
2.	Time- Delay after or before a command.
3.	GPIO access via GPIO Zero and WiringPi.
4.	ESP32 controller for Adafruit_bno055-  interface and data type for bno055 .
5.	RPI.GPIO- Configure, read, and write to GPIO pins.
6.	DTOF Lidar D300 
7.	Sony IMX219 camera module with M12 lens
8.	RGB Color Module

 
## More detail about program functions:	

Operation           :     Run ROS2 operate on Ubuntu Mate Platform :
 
IMU connection :     BNO055 -> (i2c) -> ESP32 -> (serial) -> Raspberry PI

The ROS master is used to manage all robot nodes, which can be visualised through a visual component named Robot Visualizer (Rviz). The ROS master runs on the PC. Communication between the ROS master and microcontroller board is accomplished through 802.11n networking. When the Wi-Fi boots up, it creates a hotspot. The SSH can be integrated with Raspberry Pi, and the robot can be directly controlled from a laptop. The master realises a standard function such as SLAM or navigation
## ROS Flow Chart

![image](https://github.com/Poon49/Future-Engineer-WRO2023-KMIDS-Zaa/assets/76239146/ce5bdb2f-be14-4239-b896-a951e35297d4)


### Control direction autonomous vehicle follow the game rule by:
1.	Use distance from Lidar front,left,right of Wall  for decision control direction car.

   ![image](https://github.com/Poon49/Future-Engineer-WRO2023-KMIDS-Zaa/assets/76239146/1791e17b-3fb1-49ce-a4a2-5d5b942efbf8)

   -  Forward by condition :
      Average distance left and distance right from value of Lidar.

      ![image](https://github.com/Poon49/Future-Engineer-WRO2023-KMIDS-Zaa/assets/76239146/0708fc3e-3701-4922-9941-7fd20c762cf6)

   - Turn Left/ Right under condition :
 
      If  Distance  Left     >   Distance  Right     -->   Control vehicle turn left.
      If  Distance  Right   >   Distance  Left     -->   Control vehicle turn right.
      Distance Front is control distance car with wall when turn 
2.	Accurate parking position by degree value of IMU and distance left,right at start point.
      PID Control:
      PD Control is enough for steering, As below  kp and kd for adjust Servo motor :
      PD = int((kp*error) + (kd*rateError))

### Main Functions :
1.Open CV
2.IMU function (Gyroscope)
3.Light Detection

1.Open CV
Detection of color using cv2.inRange(image, lower, upper) find out a range of pixel  ,then Lower,Upper keep in form numpy :

   #Green
   low_green=np.array([35,50,50])
   high_green=np.array([85,255,255])

   #Red
   low_red=np.array([150,100,100])
   high_red=np.array([180,255,255])

2. IMU function (Gyroscope)  
      Turn direction to be decision by IMU of Gyroscope.
   ![image](https://github.com/Poon49/Future-Engineer-WRO2023-KMIDS-Zaa/assets/76239146/f00643a4-960d-4e77-a829-d26207358349)

      
      error = heading-int(self.yaw)
   
      bring error value for adjust speed Servo motor for run to the target value, speed of Servo motor depends on error value.
4. RGB Light detection
   Lane Detection
   
   Blue Lane
   Orange Lane

   1.	We determine if the robot should turn left or right on the turn by using what lane our camera find first.
   2.	If we found blue first, then we turn left
   3.	If we found orange first, then we turn right.
   

