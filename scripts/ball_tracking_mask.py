from cvzone.ColorModule import ColorFinder
import cvzone
import cv2

cap = cv2.VideoCapture(1)
# cap.set(3, 1280)
# cap.set(4,720)

myColorFinder = ColorFinder(False)
hsvValues = {'hmin': 28, 'smin': 85, 'vmin': 107, 'hmax': 38, 'smax': 255, 'vmax': 255}

while True:
    success, img = cap.read()
    imgColor, mask = myColorFinder.update(img, hsvValues)
    imgContour, contours = cvzone.findContours(img, mask, minArea=25)
    imgStack = cvzone.stackImages([img, imgColor, mask, imgContour], 2, 0.5)

    if contours:
        data = contours[0]['center'][0], contours[0]['center'][1], contours[0]['area']

        print(data)

    cv2.imshow("Image", imgStack)
    cv2.waitKey(1)
