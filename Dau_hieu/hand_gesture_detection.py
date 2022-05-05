

import cv2
import numpy as np
from time import sleep
import mediapipe as mp
import tensorflow as tf
from tensorflow.keras.models import load_model


import RPi.GPIO as GPIO          
from time import sleep

from gpiozero import AngularServo

s = AngularServo(14, min_angle=-90, max_angle=90)

  


# initialize mediapipe
mpHands = mp.solutions.hands
hands = mpHands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mpDraw = mp.solutions.drawing_utils

# Load the gesture recognizer model
model = load_model('mp_hand_gesture')

# print(mp.version)
# Load class names
f = open('gesture.names', 'r')
classNames = f.read().split('\n')
f.close()
# if(classNames == 'okay' ) :
#     print("Di tien") 

# print(classNames)

# Initialize the webcam
cap = cv2.VideoCapture(0)
s.angle = -28


in3 = 24
in4 = 23
enB = 25

in1 = 3
in2 = 2
enA = 4

temp1 = 1

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
     

while True:
    # Read each frame from the webcam
    _, frame = cap.read()

    x, y, c = frame.shape

    # Flip the frame vertically
    frame = cv2.flip(frame, 1)
    framergb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Get hand landmark prediction
    result = hands.process(framergb)

    # print(result)
    
    className = ''

    # post process the result
    if result.multi_hand_landmarks:
        landmarks = []
        for handslms in result.multi_hand_landmarks:
            for lm in handslms.landmark:
                # print(id, lm)
                lmx = int(lm.x * x)
                lmy = int(lm.y * y)

                landmarks.append([lmx, lmy])

            # Drawing landmarks on frames
            mpDraw.draw_landmarks(frame, handslms, mpHands.HAND_CONNECTIONS)

            # Predict gesture
            prediction = model.predict([landmarks])
            # print(prediction)
            classID = np.argmax(prediction)
            className = classNames[classID]
            
            if (className == "fist") :
                print("di toi")
                
                go_to()
                
                cv2.putText(frame, "Go to", (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 0, 255), 2, cv2.LINE_AA)#
                
                
            elif(className == "stop") :
                print("dung lai")
                
                stop()
                cv2.putText(frame, "Stop", (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 0, 255), 2, cv2.LINE_AA)
                
              
            elif(className == "thumbs up"):
                print("re trai")
                
                left()
                
                cv2.putText(frame, "Left", (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                           #  1, (0, 0, 255), 2, cv2.LINE_AA)
                
              
            elif(className == "thumbs down"):
                print("re phai")
                
                right()
                
                cv2.putText(frame, "Right", (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 0, 255), 2, cv2.LINE_AA)
                
                
            elif(className == "call me"):
                print("di lui")
                
                go_back()
                
               cv2.putText(frame, "Go back", (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 0, 255), 2, cv2.LINE_AA)
               
            else :
                print("Error")
                cv2.putText(frame, "Error", (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 0, 255), 2, cv2.LINE_AA)
        #2 - peace 
    # Show the final output
    cv2.imshow("Output", frame) 

    if cv2.waitKey(1) == ord('q'):
        break

# release the webcam and destroy all active windowsvvvv
cap.release()

cv2.destroyAllWindows()