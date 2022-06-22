import re
import cv2 as cv
import time
import picamera
import numpy as np

try:
    while True:
        with picamera.PiCamera() as camera:
            camera.resolution = (320, 240)
            camera.framerate = 12
            image = np.empty((240 * 320 * 3,), dtype=np.uint8)
            camera.capture(image, 'bgr')
            image = image.reshape((240, 320, 3))
            cv.imshow('frame', image)
            key = cv.waitKey(30) & 0xff
            if key == 27:
                break
except KeyboardInterrupt:
    cv.destroyAllWindows()
    # cap.release()

# cap = cv.VideoCapture('./move.MOV')
# fourcc = cv.VideoWriter_fourcc(*'XVID')
# out = cv.VideoWriter('move.avi',fourcc, 5, (640,480))

# cap = cv.VideoCapture(0)
# # cap.set(cv.CAP_PROP_AUTO_EXPOSURE, 0.05)
# # cap.set(cv.CAP_PROP_EXPOSURE, -1) 
# # cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
# # cap.set(cv.CAP_PROP_FRAME_HEIGHT, 960)

# while True:
#     try:
#         ret, frame = cap.read()
#         # print(frame)
#         cv.imshow('frame', frame)
#         key = cv.waitKey(30) & 0xff
#         if key == 27:
#             break
#         # cv.imwrite(f'image_{i+1}.jpg', frame)
#         # print(f'write image {i}')
#         # time.sleep(1)        
    
#     except KeyboardInterrupt:
#         break

# cv.destroyAllWindows()
# cap.release()
