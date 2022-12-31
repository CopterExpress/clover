import numpy as np
import cv2
fullbody_cascade = cv2.CascadeClassifier('haarcascade_fullbody.xml') 
upperBody_cascade = cv2.CascadeClassifier('haarcascade_upperbody.xml') 
img = cv2.VideoCapture('waterrr.mp4')
while True:
    ret, frames =img.read()
    gray= cv2.cvtColor(frames, cv2.COLOR_BGR2GRAY)
    upper = upperBody_cascade.detectMultiScale(gray, 1.1, 1)
    full = fullbody_cascade.detectMultiScale(gray, 1.1 , 2)
    for (x,y,w,h) in upper:
        cv2.rectangle(frames, (x,y), (x+w, y+h), (0,0,255),2)
    for (x,y,w,h) in full:
        cv2.rectangle(frames, (x,y), (x+w, y+h), (0,0,255),2)
    cv2.imshow('image',frames)
    if cv2.waitKey(33)==22:
        break
cv2.destroyAllWindows()
