#!/usr/bin/env python3
import cv2
import numpy as np
import time
import sys
from picamera2 import Picamera2

picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": (1024, 768), "format": "RGB888"})
picam2.configure(config)

picam2.start()


    # Define lower and uppper limits
ranges = [("white", np.array([180, 180, 180]), np.array([255, 255, 255]))
    ,("blue", np.array([150, 0, 0]), np.array([255, 200, 50]))
    ,("green", np.array([0, 150, 0]), np.array([100, 255, 100]))
    ,("yellow", np.array([0, 180, 180]), np.array([50, 255, 255]))
    ,("red", np.array([0, 0, 180]), np.array([20, 20, 255]))]

while True:
    img = picam2.capture_array()
    img = cv2.medianBlur(img, 3)

    # apply morphology
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (20,20))
    img = img[170:600, 0:]
    cv2.imwrite('detected.jpg', img)

    for r in ranges:
        color = r[0]
        lower = r[1]
        upper = r[2]

        # Create mask to only select our color
        thresh = cv2.inRange(img, lower, upper)
        morph = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        # invert morp image
        mask = 255 - morph

        # apply mask to image
        result = cv2.bitwise_and(img, img, mask=mask)

        contours,hierarchy = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            c = max(contours, key = cv2.contourArea)
            x,y,w,h = cv2.boundingRect(c) # Object's size and coordinates in image.
            if w > 100 and h > 100:
                print("Color (%dx%d)" % (w, h), color)

   #            img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)

    # save results
    #cv2.imwrite('cropped.jpg', cropped_img)
    #cv2.imwrite('pills_morph.jpg', morph)
    #cv2.imwrite('pills_mask.jpg', mask)
    #cv2.imwrite('pills_result.jpg', result)

    #cv2.imshow('thresh', thresh)
    #cv2.imshow('morph', morph)
    #cv2.imshow('mask', mask)
    #cv2.imshow('result', result)
    #cv2.imshow('img', img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
