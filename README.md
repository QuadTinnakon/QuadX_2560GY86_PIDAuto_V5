QuadX_2560GY86_PIDAuto_V5
=========================
หนังสือเขียนโปรแกรมเครื่องบินสี่ใบพัด การเขียนโปรแกรม Quadrotor พื้นฐาน ,การบินอากาศยาน 4 ใบพัด ,โดรน drone  ,โดรนคอร์ดคอปเตอร์ Drone QuadCopter ,อากาศยานไร้คนขับ

![](https://cloud.githubusercontent.com/assets/9403558/5604332/6aeba6fa-93e8-11e4-8781-c0c3cdf485d3.jpg)

![](https://cloud.githubusercontent.com/assets/9403558/5851227/60b83d90-a236-11e4-82c5-efd6538aba85.jpg)

วีดีโอ

https://www.youtube.com/watch?v=w7J181fQ42M&feature=youtu.be

ทดสอบความเร็วการยกตัว

https://www.youtube.com/watch?v=w7J181fQ42M&list=UUK8SdBKqjdXghqlJf3-sZaQ

หน้าเวป 

http://quad3d-tin.lnwshop.com/product/30/auto2560-gy86

/*
project_QuadX_2560 GY86_PIDAuto_V1 

1. Automatic  Takeoff 
2. Landing
3. GPS Position Hold
by: tinnakon kheowree  

tinnakon_za@hotmail.com

tinnakonza@gmail.com

http://quad3d-tin.lnwshop.com/

https://www.facebook.com/tinnakonza

date: 22-12-2557(2014)  V.1 QuadX_2560 GY86_PIDAuto_V1

date: 28-12-2557(2014)      QuadX_2560 GY86_PIDAuto_V2   ,Write P-PID Controller

date: 28-12-2557(2014)      QuadX_2560 GY86_PIDAuto_V3   ,Ultrasonic and baro ,,Ultrasonic max 0.8 m change to Baro

date: 30-12-2557(2014)      QuadX_2560 GY86_PIDAuto_V4   ,tuning P-PID Controller

date: 31-12-2557(2014)      QuadX_2560 GY86_PIDAuto_V5   ,tuning Altitude Hold , Position Hold

support:  Board 2560  GY86

• Atmega2560

• MPU6050 Gyro Accelerometer //400kHz nois gyro +-0.05 deg/s , acc +-0.04 m/s^2

• MS561101BA Barometer

• HMC5883L Magnetometer //400kHz

• GPS BS-300

• Ultrasonic Sensor Module

• ESC Dargon 30A

• motor sunny 2212 1400kv 

Quad4-X

---------motor---------

int MOTOR_FrontL_PIN = 2;

int MOTOR_FrontR_PIN = 5;

int MOTOR_BackL_PIN = 6;

int MOTOR_BackR_PIN = 3;

----------rx-----------        

CPPM pin A8

*/

project_QuadX_2560 GY86_PIDAuto_V1  

1. Automatic  Takeoff 
2. Landing
3. GPS Position Hold
by: tinnakon kheowree  

tinnakon_za@hotmail.com

tinnakonza@gmail.com

http://quad3d-tin.lnwshop.com/

https://www.facebook.com/tinnakonza

หัวข้อที่ต้องรู้ในการเขียนโปรแกรม
=========================

บอร์ด Arduino MEGA 2560 กับ เซนเซอร์ GY-86 10DOF

อธิบาย Code QuadX_2560GY86_PIDAuto_V1

1.	การกำหนดความเร็วลูป 100 Hz, 50 Hz, 20 Hz, 10 Hz, 5 Hz, 1 Hz  
100 Hz ทำ อ่านเซนเซอร์ gyro,acc, ควบคุม PID , Filter
50 Hz ทำ อ่านค่ารีโมท
20 Hz อ่านค่า Ultrasonic ,ทำ Filter
10 Hz ทำ Automatic  Takeoff  และ Landing อ่านค่า   
Magnetometer และ Chack_Command, By Remote  idle-up settings 0,1,2
5 Hz คำนวณ GPS ควบคุมต่ำแหน่ง 
1 Hz แสดงสถานะหลอดไฟ LED ทำ Accelerometers trim โดยใช้ รีโมท 

2.	การสร้างตัวแปรเพื่อเก็บค่า  int 16 bit, long 32 bit ,uint8_t, unsigned long , float 32bit,
3.	การอ่านค่าเซนเซอร์ gyroscope , accelerometer, magnetometer
4.	การอ่านค่าบาโรเพื่อมาคำนวณค่าความสูงจากเซนเซอร์ MS5611 
5.	การทำตัวสังเกตเพื่อประมาณค่าความสูง  ,State estimation with Kalman Filter
6.	การอ่านค่าเซนเซอร์ Ultrasonic , Trigger, Echo
7.	การอ่านค่าเซนเซอร์ GPS ใช้ NMEA_BINARY settings ของ MultiWii
8.	การอ่านค่ารีโมท PWM 1-1.9 ms  แบบ CPPM สายเส้นเดียว ได้ 6 CH
9.	การกำหนดค่าเริ่มต้นใน setup()
-	ค่าเริ่มต้นการอ่านรีโมท
-	ค่าเริ่มต้นสั่งมอเตอร์
-	ค่าเริ่มต้น GPS
-	ค่าเริ่มต้น I2C เปลี่ยน I2C clock rate to 400kHz
-	ค่าเริ่มต้นเซนเซอร์ GY-86 , mpu6050  HMC5883 MS561101BA
-	ค่าเริ่มต้นอัลตร้าโซนิค
-	การอ่านเซนเซอร์เริ่มต้น
-	การทำคาริเบตเซนเซอร์
-	ตั้งค่าเริ่มต้นการหามุม AHRS (An attitude and heading reference system , AHRS)
-	กำหนดเวลาเริ่มต้น
10.	การทำ sensor Calibration และการแปลงหน่วย rad/s , deg/s , m/s^2
11.	การลดสัญญานรบกวน sensor , Moving average  filter , Low pass  filter
12.	วิธีการหาค่ามุม AHRS, Quaternion , direction cosine matrix , Euler angles,  Rotation matrix  Body Fixed Frame, Earth-Fixed frame
13.	การอ่านค่า GPS latitude and longitude , ความเร็วการเคลื่อนที่ Earth-Fixed frame
14.	การควบคุม PIDA , P-PID  , Roll, Pitch, Yaw
-	คำนวณหาค่ามุมรีโมท
-	การดิฟหาความเร็วจากมุมรีโมท
-	การคำนวณค่า error
-	การคำนวณกฏการควบคุมมุมและ Rate Gyro
15.	การควบคุมความสูงใช้ State feedback control หรือ PID 
16.	เทคนิคการจูนค่า Gain ที่ใช้ควบคุม
17.	เงื่อนไขการทำ Automatic  Takeoff และ Landing
18.	การควบคุมต่ำแหน่ง GPS ใช้ PID control
19.	การทำ Motor mixing theory,  Quad +,x
20.	การสั่ง PWM 1-1.9 ms ให้มอเตอร์หมุน
21.	การทำ Electronic Speed Controllers Calibration 1 - 1.9 ms
22.	การทำ Accelerometers trim โดยใช้ รีโมท
23.	การทำ Calibration sensor Magnetometer หาค่า Max , Min
24.	การทำ ARM and DISARM มอเตอร์ไม่หมุนและหมุน

by: ทินกร เขียวรี  tinnakon kheowree  

tinnakon_za@hotmail.com

tinnakonza@gmail.com

http://quad3d-tin.lnwshop.com/

https://www.facebook.com/tinnakonza
