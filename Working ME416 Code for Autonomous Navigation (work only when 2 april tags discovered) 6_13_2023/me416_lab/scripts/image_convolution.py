#!/usr/bin/env python
import cv2
import numpy as np

img=cv2.imread('BU_logo.png',cv2.IMREAD_COLOR) #by default, imread reads the image in BGR format

img_gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


kernel_vertical=np.array([[-1,1]])
kernel_horizontal=np.array([[-1],[1]])

img_filter_vertical = cv2.filter2D(img_gray,-1,kernel_vertical)
img_filter_horizontal = cv2.filter2D(img_gray,-1,kernel_horizontal)

#show the original image
cv2.imshow('image',img)
cv2.imshow('filtered vertical',img_filter_vertical)
cv2.imshow('filtered horizontal',img_filter_horizontal)


cv2.waitKey(00) # wait for a  key or 5000ms (5s)
cv2.destroyAllWindows() #close all windows
