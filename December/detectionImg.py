import numpy as np
import cv2



imgg = cv2.imread('123.jpg',cv2.IMREAD_COLOR)
img= cv2.resize(imgg,(1280,720))





fullbody_cascade = cv2.CascadeClassifier('haarcascade_fullbody.xml') 
upperBody_cascade = cv2.CascadeClassifier('haarcascade_upperbody.xml') 





 




gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

upper = upperBody_cascade.detectMultiScale(gray, 1.1, 1)
full = fullbody_cascade.detectMultiScale(gray, 1.1 , 2)



    
for (x,y,w,h) in upper:
    cv2.rectangle(img, (x,y), (x+w, y+h), (60,70,60),2)
for (x,y,w,h) in full:
    cv2.rectangle(img, (x,y), (x+w, y+h), (60,70,60),2)





    
cv2.imshow('image',img)
cv2.waitKey(0) 
cv2.destroyAllWindows()