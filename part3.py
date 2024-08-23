import time
import cvzone
import cv2
import RPi.GPIO as GPIO
import threading
from multiprocessing.pool import ThreadPool
import multiprocessing as mp
from cvzone.ClassificationModule import Classifier

servo1PIN=27
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo1PIN,GPIO.OUT)
GPIO.setwarnings(False)
p1=GPIO.PWM(servo1PIN,50)
servo_flag=0

class ClassifierThread(threading.Thread):
    def __init__(self,classifier):
        threading.Thread.__init__(self)
        self.classifier=classifier
        self.frame=None
        self.predictions=None
        self.index=None
        self.running=True
    def run(self):
        while self.running:
            if self.frame is not None:
                self.predictions, self_index = self.classifier.getPrediction(self.frame)
    def stop(self):
        self.running=False
        
cap=cv2.VideoCapture(0)
classifier=Classifier('/home/gtukuzgun/Desktop/aplani/model/keras_model.h5','/home/gtukuzgun/Desktop/aplani/model/labels.txt')
classifier_thread=ClassifierThread(classifier)
classifier_thread.start()

while True:
    print("part2 calisiyor")
    if servo_flag==1:
        
        print("servo acildi")
        time.sleep(5)
        
    
    _, img=cap.read()
    classifier_thread.frame=img
    
    
    #print(classifier_thread.predictions[1])
    if classifier_thread.predictions is not None:
        print(classifier_thread.predictions)
        no_target=classifier_thread.predictions[0]
        insan=classifier_thread.predictions[1]
        asfalt=classifier_thread.predictions[2]
        
        
        
        
        if insan>=0.80 and servo_flag==0:
            p1.start(9)
            time.sleep(1)
            p1.ChangeDutyCycle(0)
            time.sleep(0.5)
            p1.stop
            servo_flag=1
            #subprocess.call(["python3","/home/gtukuzgun/Desktop/KUZGUN2023_v11.py"])
            
            
            
    #cv2.imshow("Image",img)
    key=cv2.waitKey(1)
    
classifier_thread.stop()
classifier_thread.join()
cap.release()
cv2.destroyAllWindows()
