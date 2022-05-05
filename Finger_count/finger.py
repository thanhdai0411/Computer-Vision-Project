import cv2
import time
import os
import HandTrackingModule as htm

import RPi.GPIO as GPIO          
from time import sleep

from gpiozero import AngularServo

s = AngularServo(14, min_angle=-90, max_angle=90)

  


wCam, hCam = 300, 300

cap = cv2.VideoCapture(0)

cap.set(3, wCam)
cap.set(4, hCam)
#---------------

s.angle = -28


in3 = 24
in4 = 23
enB = 25

in1 = 3
in2 = 2
enA = 4


GPIO.setmode(GPIO.BCM)

GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(enA, GPIO.OUT)
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)


GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(enB, GPIO.OUT)
GPIO.output(in3, GPIO.LOW)
GPIO.output(in4, GPIO.LOW)


p = GPIO.PWM(enA, 1000)
o = GPIO.PWM(enB, 1000)

p.start(30)
o.start(30)


def go_to():
    
    s.angle = -28
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)

    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    
    


def go_back():
    s.angle = -28
    sleep(1)
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)

    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)

    
def left():
    
    
    s.angle = -90
    sleep(1)
    
    
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)

    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    
    

def right():
    
    
    s.angle = 90
    sleep(1)
    
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)

    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)


def stop():
    s.angle = -28
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)

    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)

pTime = 0


detector = htm.handDetector(detectionCon=0.75)

tipIds = [4, 8, 12, 16, 20]

while True:
    success, img = cap.read()
    img = detector.findHands(img)
    lmList = detector.findPosition(img, draw=False)
    # print(lmList)

    if len(lmList) != 0:
        fingers = []

        # Thumb
        if lmList[tipIds[0]][1] > lmList[tipIds[0] - 1][1]:
            fingers.append(1)
        else:
            fingers.append(0)

        # 4 Fingers
        for id in range(1, 5):
            if lmList[tipIds[id]][2] < lmList[tipIds[id] - 2][2]:
                fingers.append(1)
            else:
                fingers.append(0)

        # print(fingers)
        totalFingers = fingers.count(1)
        print(totalFingers)
        if(totalFingers == 1) :
            print("Forward")
            go_to()
            cv2.putText(img, "Forward", (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                             1, (0, 0, 255), 2, cv2.LINE_AA)
        elif(totalFingers == 2) :
            print("Backward")
            go_back()
            cv2.putText(img, "Backward", (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                             1, (0, 0, 255), 2, cv2.LINE_AA)
        elif(totalFingers == 3) :
            print("Left")
            left() 
            # cv2.putText(img, "Left", (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            # 1, (0, 0, 255), 2, cv2.LINE_AA)
        elif(totalFingers == 4) :
            print("Right")
            right()
            cv2.putText(img, "Right", (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 0, 255), 2, cv2.LINE_AA)
        elif(totalFingers == 5) :
            print("Stop")
            stop()
            cv2.putText(img, "Stop", (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 0, 255), 2, cv2.LINE_AA)
        else:
            cv2.putText(img, "Error", (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 0, 255), 2, cv2.LINE_AA)
        # h, w, c = overlayList[totalFingers - 1].shape
        # img[0:h, 0:w] = overlayList[totalFingers - 1]

        # cv2.rectangle(img, (20, 225), (170, 425), (0, 255, 0), cv2.FILLED)
        # cv2.putText(img, str(totalFingers), (45, 375), cv2.FONT_HERSHEY_PLAIN,
        #             10, (255, 0, 0), 25)

    

    cv2.imshow("Image", img)
    cv2.waitKey(1)
