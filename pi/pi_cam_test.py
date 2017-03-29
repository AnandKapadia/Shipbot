import numpy as np
import cv2 as cv
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

def load_image(file_name):
	img = cv.imread(file_name, 3)
	return img;

def threshold(img, thresh, th_max, th_type):
	_, img2 = cv.threshold(img, thresh, th_max, th_type);
	return img2

def range_threshold(img, min, max):
	img2 = cv.inRange(img, min, max);
	return img2

def range_threshold_color(img, rmin, rmax, gmin, gmax, bmin, bmax):
	r = img[:, :, 2]
	r = cv.inRange(r, rmin, rmax)
	#display_image(r)
	#cv.waitKey(0)
	g = img[:, :, 1]
	g = cv.inRange(g, gmin, gmax)
	#display_image(g)
	#cv.waitKey(0)
	b = img[:, :, 0]
	b = cv.inRange(b, bmin, bmax)
	#display_image(b)
	#cv.waitKey(0)
	img2 = cv.merge((b, g, r))
	#display_image(img2)
	#cv.waitKey(0)
	return img2

def grayscale(img):
	img2 = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
	return img2

def gaussian_blur(img, radius):
	img2 = cv.GaussianBlur(img,(radius,radius),0)
	return img2

#give this function a size of the form (x, y)
def resize(img, r_tuple):
	img2 = cv.resize(img, r_tuple, fx=f_x, fy=f_y)
	return img2

def resize2(img, f_x, f_y):
	img2 = cv.resize(img, (0,0), fx=f_x, fy=f_y)
	return img2
def display_image(img):
	cv.imshow('image', img)
	cv.waitKey(1)

	return;



def main():
        
        garden = 1
        breaker = 0
	camera = PiCamera()

	camera.resolution=(640, 480)
	camera.framerate = 32
	rawCapture = PiRGBArray(camera, size=(640, 480))
	time.sleep(0.1)
	# for i in range(1, 8):
	# 	#load image
	# 	limg = load_image("test_pics/lv_" + str(i) + ".jpg");
	
#	for i in range(1, 2):
		#load image
#		limg = load_image("test_pics/gv_" + str(i) + ".JPG");
        bool_first = 0
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):		
		limg = frame.array
		framing_offset = 50
		limg = limg[framing_offset:640-framing_offset, 0:480]
		raw_img = limg.copy()
		
		raw_img = gaussian_blur(raw_img, 5)

		#color detect
		if(breaker):
                        raw_img = range_threshold_color(raw_img, 170, 255, 60, 135, 20, 95)
                else:
                        raw_img = range_threshold_color(raw_img, 0, 40, 20, 90, 75, 135)

		#grayscale
                        
		raw_img = grayscale(raw_img)

		#threshold
		raw_img = threshold(raw_img, 254, 255, cv.THRESH_BINARY)
		
                if(breaker):
                        raw_img = gaussian_blur(raw_img, 15)
                        
		temp, contours, hierarchy = cv.findContours(raw_img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

		img3 = cv.merge((raw_img, raw_img, raw_img))

		cv.drawContours(img3, contours, -1, (0,255,0), 3)

                maxcnt = 0
		foundcnt = 0
		maxa = 0
		if(breaker == 0):
			foundcnt2 = 1
			foundcnt3 = 1
                for cnt in contours:
			x,y,w,h = cv.boundingRect(cnt)
			if(x < 15 or y < 15):
				continue
			area = w * h
			if(area > maxa):
				foundcnt = 1
				maxa = area
				maxcnt = cnt

                if foundcnt:
                        x,y,w,h = cv.boundingRect(maxcnt)
                        cv.rectangle(img3,(x,y),(x+w,y+h),(0,0,255),2)
                        cv.rectangle(limg,(x,y),(x+w,y+h),(0,0,255),2)
                        cv.circle(limg, (x+w/2, y+h/2), 3, (255, 255, 255), thickness=5, lineType=8, shift=0) 
                        cv.circle(img3, (x+w/2, y+h/2), 3, (255, 255, 255), thickness=5, lineType=8, shift=0) 

                        temp = (min(w, h) * 1.0)/max(w, h)
                        new_img = limg.copy()
                        #new_img = grayscale(new_img)
                        color_1 = 0
                        color_2 = 0;
                        if(x+w > 600 or x < 50 or y+h > 450 or y < 50):
                                display_image(limg)
                                rawCapture.truncate(0)
                                continue
                        print temp
                        if(not breaker):
                                if(garden):
                                        print "\nType: garden"
                                        if(temp > 0.65):
                                                print "Orientation: facing"
                                        else:
                                                print "Orientation: not facing"
                                else:			
                                        print "\nType: linear"
                                        if(temp > .225):
                                                print "Orientation: facing"
                                        else:
                                                print "Orientation: not facing"


                                if(not garden):
                                        if(max(h, w) == w):
                                                color_1 = new_img[y+h/2, x-10]
                                                color_2 = new_img[y+h/2, x+w+10]
                                        else: 
                                                color_1 = new_img[y -10, x+w/2]
                                                color_2 = new_img[y+h+10, x+w/2]
                                        if(max(h,w) == w):
                                                if(color_1[0] < color_2[0]):
                                                        print "Joint to left side"
                                                else: 
                                                        print "Joint to right side"
                                        else:
                                                if(color_1[0] < color_2[0]):
                                                        print "Joint to top side"
                                                else: 
                                                        print "Joint to bottom side"
		#display_image(img3)
		display_image(limg)
		#cv.waitKey(0)
		rawCapture.truncate(0)	
	#display image
	#cv.destroyAllWindows()


if __name__ == "__main__":
    main()
