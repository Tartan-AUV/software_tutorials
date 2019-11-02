import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

def template_matching(src, template):
    frame = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    img = frame.copy()
    w, h = template.shape
    method_str = 'cv.TM_CCOEFF_NORMED'
    method = eval(method_str)
    for i in range(4):
        resize_img = cv.resize(img, None, fx=1/(2**(0.5*i)), fy=1/(2**(0.5*i)))
        result = cv.matchTemplate(resize_img, template, method)
        loc = np.where(result >= 0.75)
        for pt in zip(*loc[::-1]):
            x = (pt[0]*int(2**(0.5*i)), pt[1]*int(2**(0.5*i)))
            y = (pt[0]*int(2**(0.5*i)) + w, pt[1]*int(2**(0.5*i)) + h)
            cv.rectangle(src, x,y, (255,0,0), 1)
    cv.imshow("Matching Result", result)
    cv.imshow("Detected Image", img)
cap = cv.VideoCapture('test480.mp4')
template = cv.imread('red.jpg')
template = cv.cvtColor(template, cv.COLOR_BGR2GRAY)

while(True):
    ret, frame = cap.read()

    template_matching(frame, template)
    cv.imshow('frame', frame)
    cv.waitKey(100)

cap.release()
cv.destroyAllWindows()
