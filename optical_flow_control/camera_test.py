import re
import cv2 as cv

cap = cv.VideoCapture(0)

while True:
    try:
        ret, frame = cv.read()
        if ret:
            cv.imshow('frame', frame)
    
    except KeyboardInterrupt:
        break