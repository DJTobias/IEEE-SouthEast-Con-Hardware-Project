#!/usr/bin/env python2
# Original http://pastebin.com/C4r2uNCC

# Modified By : Daniel Tobias, Lowell Bass, Ryan Dellana
# Dependency :
# http://zbar.sourceforge.net/


import cv2.cv as cv #Use OpenCV-2.4.3
import cv2
import zbar
import numpy as np

def scanner_proccess(frame,set_zbar):    
    set_width = 20.0 / 100
    set_height = 30.0 / 100
    coord_x = int(frame.width * (1 - set_width)/2)
    coord_y = int(frame.height * (1 - set_height)/2)
    _x = coord_x
    _y = coord_y 
    width = int(frame.width * set_width)
    height = int(frame.height * set_height)
    get_sub = cv.GetSubRect(frame, (coord_x-230, coord_y-80, width-1, height-1))
    center = [_x,_y]
    cm_im = cv.CreateImage((get_sub.width, get_sub.height), cv.IPL_DEPTH_8U, 1)
    cv.ConvertImage(get_sub, cm_im)
    image = zbar.Image(cm_im.width, cm_im.height, 'Y800', cm_im.tostring())
    set_zbar.scan(image)
    for symbol in image:
	    # topLeftCornersx,topLeftCornersy, bottomLeftCornersx,bottomLeftCornersy, 
        # bottomRightCornersx,bottomRightCornersy, topRightCornersx,topRightCornersy = symbol.location           
	    print '\033[1;32mResult : %s symbol1 "%s" \033[1;m' % (symbol.type,symbol.data) 
        # print '\033[1;32mResult : %s symbol1 "%s" \033[1;m' % (symbol.location,9)
	    l =[[],[],[],[]]	
	    l =symbol.location 
	    print l
    	width = (l[0][0] +l[1][0] +l[2][0] +l[3][0])/4
        hieght = (l[0][1] +l[1][1] +l[2][1]+l[3][1]) /4
    	center = [_x+abs(width),_y+abs(hieght)]
        if len(l[0]) >0:
    	    cv.Circle(frame,(center[0],center[1]), 5, (100,56,255), -1)
    	print center
    cv.Rectangle(frame, (coord_x, coord_y), (coord_x + width, coord_y + height), (255,0,0))
    # cv.Circle(frame,(center[0],center[1]), 20, (100,56,255), -1)
    cv.ShowImage("webcam2", frame)
    cv.ShowImage("webcam3", cm_im)
    cv.WaitKey(10)

def scanner_procces2(frame2,set_zbar):    
    set_width = 20.0 / 100
    set_height = 30.0 / 100
    coord_x = int(frame2.width * (1 - set_width)/2)
    coord_y = int(frame2.height * (1 - set_height)/2)
    # coord_y +=100
    width = int(frame2.width * set_width)
    height = int(frame2.height * set_height)
    # height+=100
    # get_sub = cv.GetSubRect(frame2, (coord_x-230, coord_y+100, width+10, height-20))
    # cv.Rectangle(frame2, (coord_x, coord_y), (coord_x + width, coord_y + height), (255,0,0))
    # cv.Rectangle(frame2, (coord_x, coord_y+100), (coord_x + width, coord_y + height+120), (255,0,211))
    cm_im = cv.CreateImage((frame2.width, frame2.height+100), cv.IPL_DEPTH_8U, 1)
    cv.ConvertImage(frame2, cm_im)
    image = zbar.Image(cm_im.width, cm_im.height, 'Y800', cm_im.tostring())
    set_zbar.scan(image)
    for symbol in image:
        print '\033[1;32mResult : %s symbol2 "%s" \033[1;m' % (symbol.type,symbol.data)
    cv.Rectangle(frame2, (coord_x, coord_y+100), (coord_x-300 + width, coord_y + height+120), (255,0,211))    
    cv.ShowImage("webcam", cm_im)
    cv.ShowImage("webcam2", frame2)
    cv.WaitKey(10)
    
def flip_image(img):
    return cv2.flip(img,1)
	# fimg=cv2.flip(img,0)

def rotate_image(img,deg):
	(h, w) = img.shape[:2]
	center = (w / 2, h / 2)
    # rotate the image by 180 degrees
	M = cv2.getRotationMatrix2D(center, 90, 1.0)
	rotated = cv2.warpAffine(img, M, (w, h))
	return cv2.flip(img,1)

if __name__ == "__main__":
    # set up our stuff
    cv.NamedWindow("webcam", cv.CV_WINDOW_AUTOSIZE)
    cv.NamedWindow("webcam2", cv.CV_WINDOW_AUTOSIZE)
    cv.NamedWindow("webcam3", cv.CV_WINDOW_AUTOSIZE)
    capture = cv.CaptureFromCAM(-1)
    set_zbar = zbar.ImageScanner()
    while True:
        frame = cv.QueryFrame(capture)
        scanner_proccess(frame, set_zbar)
