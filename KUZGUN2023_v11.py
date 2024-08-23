import cv2
import numpy as np
import imutils
import math
import time
import subprocess
import RPi.GPIO as GPIO
from dronekit import connect,Vehicle,LocationGlobalRelative
import sys
import os

temp_d_x_cm=-1000


# ayarlamalar
cap = cv2.VideoCapture(0)


servo1PIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo1PIN, GPIO.OUT)
p1 = GPIO.PWM(servo1PIN, 50)


time.sleep(10)

connection_string= "/dev/ttyACM0"
baud_rate=57600
vehicle=connect(connection_string,baud=baud_rate,wait_ready=True)


ft=time.time()
#vurus_dist = 20
time_counter= time.time()
servo_sure= 10 #servoların devreye girmesi için gereken süre saniye

#                       h  s   v
lower_red = np.array([161, 155, 84]) 
upper_red= np.array([179, 255, 255])
# kamera çözünürlüğü #değiştirmeyiniz
coz_x = 1280 
coz_y = 720

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)


#sabitler


g=980.665 #cm/s

mini_map_oran=math.sqrt((200*100)/(coz_x*coz_y))

servo1renk= (0,0,255)
servo2renk= (0,0,255)

#kamera özellikleriyle alakalı sabitler
y_cam_sabiti= (13.5,9.8)  #(h,2a)    19.95 derece açı
x_cam_sabiti= (13.5,14.5) #(h,2a)    28.24 derece açı
#
servo_flag=0
servo2_flag=0
cooldown=0
cooldown_counter=0
#28.24 derece açıyla konduğunda 56.48 derece kamera açıklığındaki alanı görür
tan_x= 1.85 # tan56.48
tan_y= (y_cam_sabiti[1]/2)/y_cam_sabiti[0] #kamera x doğrultusunda eğimli konduğu için y yi yarım açı almak gerekir


# kare kare işleyen döngü
while True:

    #ekran kaydi
    
    
    #gerçek zamanlı hesaplamalar için kodun işlenme süresi hesabı. Son satırlarda ft yi bulabilirsiniz
    st= time.time()
    execute_time= (st-ft)*1000 #dogrusu bu sekilde
    print(execute_time)

    # her kare için ayarlamalar
    _, frame = cap.read()  # kameradan kare alır
    frame= cv2.flip(frame,1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_red, upper_red)

    cnts = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    cv2.arrowedLine(frame, (1200, 360), (1280,360), (0,0,0), thickness=2)
    cv2.putText(frame, f"ucus yonu", (1200, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 0, 0), 2)

    gecen_sure=round(time.time()-time_counter,2)
    cv2.putText(frame, f"{gecen_sure}", (1100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(25, 20, 255), 2)
    
    if gecen_sure<servo_sure:
        
        cv2.putText(frame, f"Servolar Devre Disi {round(servo_sure-gecen_sure,2)}", (1000, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(25, 20, 255), 2)
    else:
        cv2.putText(frame, f"Servolar Aktif", (1100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(25, 200, 55), 2)
    

    for c in cnts:

        area = cv2.contourArea(c)
        

        if area > 1500:
            
            
            cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)

            
            # merkez koordinatları
            M = cv2.moments(c)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            vehicle.wait_ready('autopilot_version')


            K= vehicle.location.global_relative_frame.alt
            K= K*100 #cm e cevirmek icin
            
            if K<0:
                K=-1*K


            #gördüğü max uzaklık 
            x_max_cm= K*tan_x
            y_max_cm= K*tan_y*2
           

            #cm cinsinden nişangaha uzaklık
            crosshair1 = (0, int(coz_y/2))
            #x
            d_x= abs(cx-crosshair1[0])
            d_x_cm=(x_max_cm)*(d_x/(coz_x)) #distance x cm
            #y
            d_y= abs(cy-crosshair1[1])
            d_y_cm= (y_max_cm)*(d_y/(coz_y/2)) #cismin yatay mesafesi


            #reel_hiz = vehicle.groundspeed

            delta_d_x_cm= d_x_cm-temp_d_x_cm

            bagil_hiz= delta_d_x_cm/(execute_time/1000) #cm/s

            temp_d_x_cm=d_x_cm

            #düşme noktasi hesaplama cm
            #dusme_noktasi_x= abs(reel_hiz*(math.sqrt((2*K)/g))) 
            dusme_noktasi_x= abs(bagil_hiz*(math.sqrt((2*K)/g))) 
            
            #gerektiginde acilabilir
            #dusme_noktasi_y= reel_hiz_y*math.sqrt((2*K)/g)

            # hedefin merkezine daire
            cv2.circle(frame, (cx, cy), 5, (255, 200, 255), -1)
            cv2.circle(frame, crosshair1, 5, (0, 0, 255), 1)

            
            #minimap
            cv2.putText(frame, f"max x:{round(x_max_cm,2)}cm", (100, 490), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(200, 255, 34), 2)
            cv2.arrowedLine(frame, (10, 500), (210,500), (0, 0, 0), thickness=2,tipLength = 0.07)

            cv2.putText(frame, f"max y:{round(y_max_cm,2)}cm", (10, 610), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(200, 255, 34), 2)
            cv2.arrowedLine(frame, (10, 500), (10,600), (0, 0, 0), thickness=2,tipLength = 0.1)         

            cv2.circle(frame, (10,550), 2, (0, 0, 255), 2)

            
            mini_cx= mini_map_oran*cx
            mini_cx+=10
            mini_cy=mini_map_oran*cy
            mini_cy+=500
            cv2.circle(frame, (int(mini_cx),int(mini_cy)), 2, (5, 255, 5), 2)

            # hedef merkezle istenilen noktaya çizgi
            cv2.line(frame, (cx, cy), crosshair1, (105, 200, 150), thickness=2, lineType=3)

            merkeze_uzaklik1 = ((cx - crosshair1[0]) ** 2) + ((cy - crosshair1[1]) ** 2)
            merkeze_uzaklik1 = math.sqrt(merkeze_uzaklik1)

            # orta nokta (çizgi üstüne uzaklık yazmak için)
            ort_x = (cx + crosshair1[0]) / 2
            ort_y = (cy + crosshair1[1]) / 2
            cv2.putText(frame, f"UZAKLIK : {int(merkeze_uzaklik1)}px * x{round(d_x_cm,3)}cm * y{round(d_y_cm,3)}cm", (int(ort_x) - 50, int(ort_y)),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            cv2.putText(frame, f"Servo1 released:{servo_flag}", (500, 580), cv2.FONT_HERSHEY_SIMPLEX, 0.6, servo1renk, 2)
            cv2.putText(frame, f"Servo2 released:{servo2_flag}", (500, 600), cv2.FONT_HERSHEY_SIMPLEX, 0.6, servo2renk, 2)

            cv2.putText(frame, "merkez", (cx - 20, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)
            cv2.putText(frame, f"merkez koordinatlari:({cx},{cy})", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(121, 31, 180), 2)
            cv2.putText(frame, f"bagil_hiz:{round(bagil_hiz,2)}cm/s", (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0, 255, 255), 2)
            cv2.putText(frame, f"yukseklik:{round(K,3)}cm ", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(frame, f"Yatay Dusme mesafesi:{round(dusme_noktasi_x,3)}cm", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0, 255, 255), 2)
            cv2.putText(frame, f"nisangaha uzaklik:{round(d_x_cm,2)}cm", (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0, 255, 255), 2)
        

            
            #atış şartları
            if ((d_x_cm<dusme_noktasi_x) and gecen_sure>servo_sure): #y düzlemi hesaba katılmıyor
                
                if(servo_flag==0):

                    cap.release()
                    p1.start(2.5)
                    time.sleep(0.5)
                    p1.ChangeDutyCycle(7.5)
                    time.sleep(0.5)
                    p1.stop
                    servo_flag=1
                    servo1renk= (0,255,0)
                    os.environ['OPENCV_VIDEOIO_PRIORITY_MSMF']='0'
                    time.sleep(1)
                    subprocess.call(["python3","/home/gtukuzgun/Desktop/aplani/part3.py"])
                    time.sleep(4)
                    vehicle.close()
                    
                    cv2.destroyAllWindows
                    sys.exit()
                    
                    
                    
                    

                    cv2.putText(frame, "ATES EDILDI SERVO2", (600, 530), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if(servo_flag==1 and servo2_flag!=1):
                cooldown=gecen_sure-cooldown_counter
                cv2.putText(frame, f"Servo Cooldown:{round(10-cooldown,1)}", (600, 560), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    

    ft=time.time()

    # hazırlanan bütün görüntüleri ekrana verir
    #cv2.imshow("KUZGUN2023_v1.1", frame)


    # fps ayarı 'esc' basınca duruyor
    if cv2.waitKey(1) and 0xFF==27: #değiştirilmemesi önerilir
        break

vehicle.close()
cap.release()
cv2.destroyAllWindows
