from cvzone.ColorModule import ColorFinder
import cvzone
import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(3, 1280)
cap.set(4,720)

myColorFinder = ColorFinder(True)
hsvValues = {'hmin': 28, 'smin': 85, 'vmin': 107, 'hmax': 38, 'smax': 255, 'vmax': 255}

# frame = cv2.imread('Blur.png')

while True:
    success, frame = cap.read()
    imgColor, mask = myColorFinder.update(frame, hsvValues)
    imgContour, contours = cvzone.findContours(frame, mask, minArea=40)
    imgStack = cvzone.stackImages([frame, imgColor, mask, imgContour], 2, 0.5)

    if contours:
        data = contours[0]['center'][0], contours[0]['center'][1], contours[0]['area']
        print(data)

    cv2.imshow("Image", imgStack)
    cv2.waitKey(1)
