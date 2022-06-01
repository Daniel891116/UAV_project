import re
import cv2 as cv
import time

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_AUTO_EXPOSURE, 0.05)
cap.set(cv.CAP_PROP_EXPOSURE, -1) 
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 960)

while True:
    try:
        ret, frame = cap.read()
        # print(frame)
        cv.imshow('frame', frame)
        key = cv.waitKey(30) & 0xff
        if key == 27:
            break
        # cv.imwrite(f'image_{i+1}.jpg', frame)
        # print(f'write image {i}')
        # time.sleep(1)        
    
    except KeyboardInterrupt:
        break

cv.destroyAllWindows()
cap.release()
