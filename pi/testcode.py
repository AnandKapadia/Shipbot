import numpy as np
import cv2 as cv



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
	g = img[:, :, 1]
	g = cv.inRange(g, gmin, gmax)
	#display_image(g)
	b = img[:, :, 0]
	b = cv.inRange(b, bmin, bmax)
	#display_image(b)
	img2 = cv.merge((b, g, r))
	#display_image(img2)
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

def canny(img, t1, t2):
	img2 = cv.Canny(img,t1,t2)
	return img2


def hough_lines(img, dispimg, thresh, minLL, maxLG):
	baseimg = cv.merge((img, img, img))
	minLineLength = minLL
	maxLineGap = maxLG
	lines = cv.HoughLinesP(img,0.02,np.pi/500,thresh,minLineLength , maxLineGap)
	if lines is None:
		return dispimg, img
	print len(lines[0])
	for x1,y1,x2,y2 in lines[0]:
		cv.line(dispimg,(x1,y1),(x2,y2),(0,255,0),2)
		cv.line(baseimg,(x1,y1),(x2,y2),(0,255,0),2)
	return dispimg, baseimg

def display_image(img):
	cv.imshow('image', img)
	cv.waitKey(0)
	return;

def hough_circles(img, dispimg):
	baseimg = cv.merge((img, img, img))
	circles = cv.HoughCircles(img,cv2.CV_HOUGH_GRADIENT,1,20, 50,param2=30,minRadius=0,maxRadius=0)
	if circles is None:
		return dispimg, baseimg
	circles = np.uint16(np.around(circles))
	for i in circles[0,:]:
		# draw the outer circle
		cv.circle(baseimg,(i[0],i[1]),i[2],(0,255,0),2)
		# draw the center of the circle
		cv.circle(dispimg,(i[0],i[1]),2,(0,0,255),3)
	display_image(baseimg)
	return dispimg, baseimg

def hough():
	# #canny edge detect
	# raw_img = canny(raw_img, 100, 100)
	# display_image(raw_img)

	# #gauss blur
	# raw_img = gaussian_blur(raw_img, 31)
	# display_image(raw_img)   

	# #threshold
	# raw_img = threshold(raw_img, 254, 255, cv.THRESH_BINARY)
	# display_image(raw_img)
	# 
	# #hough lines
	# raw_img, raw_img2 = hough_lines(raw_img, limg, 20, 1000, 50)
	# #raw_img, raw_img2 = hough_circles(raw_img, limg)
	# display_image(raw_img2)
	# display_image(raw_img)
	return

def blobs(): 
	#params of blob detection
	params = cv.SimpleBlobDetector_Params()
	params.filterByArea = True 
	params.filterByCircularity = False
	params.filterByConvexity = False
	params.filterByInertia = False
	# Set up the detector with default parameters.
	detector = cv.SimpleBlobDetector(params)
	 
	# Detect blobs.
	keypoints = detector.detect(raw_img)
	# Draw detected blobs as red circles.
	# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
	im_with_keypoints = cv.drawKeypoints(raw_img, keypoints, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	lim_with_keypoints = cv.drawKeypoints(limg, keypoints, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	display_image(im_with_keypoints);
	display_image(lim_with_keypoints);

def hsv_thresh(img):
	img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
	h = img[:, :, 2]
	h = cv.inRange(h, 25, 117) #46 - 107
	s = img[:, :, 1]
	s = cv.inRange(s, 120, 240)#128-230
	v = img[:, :, 0]
	v = cv.inRange(v, 90, 125)	#102-115
	
	img2 = cv.merge((h,s,v))
	display_image(img2)

	return img2
def main():
	# for i in range(1, 8):
	# 	#load image
	# 	limg = load_image("test_pics/lv_" + str(i) + ".jpg");
	for i in range(1, 7):
		#load image
		limg = load_image("test_pics/gv_" + str(i) + ".JPG");
		raw_img = limg.copy()
		display_image(raw_img)

		#gaussian blur
		raw_img = gaussian_blur(raw_img, 3)
		#display_image(raw_img)

		#color detect
		#raw_img = range_threshold_color(raw_img, 0, 30, 20, 80, 50, 125)
		raw_img  = hsv_thresh(raw_img)
		display_image(raw_img)

		#grayscale
		raw_img = grayscale(raw_img)
		#display_image(raw_img)

		#threshold
		raw_img = threshold(raw_img, 254, 255, cv.THRESH_BINARY)
		#display_image(raw_img)

		raw_img = gaussian_blur(raw_img, 25)
		#display_image(raw_img)

		#threshold
		raw_img = threshold(raw_img, 20, 255, cv.THRESH_BINARY)
		display_image(raw_img)

		contours, hierarchy = cv.findContours(raw_img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
		#temp, contours, hierarchy = cv.findContours(raw_img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

		img3 = cv.merge((raw_img, raw_img, raw_img))

		cv.drawContours(img3, contours, -1, (0,255,0), 3)

		maxcnt = 0
		maxa = 0;
		for cnt in contours:
			x,y,w,h = cv.boundingRect(cnt)
			if(x < 10 or y < 10):
				continue
			if((w*h) > maxa):
				maxa = w*h
				maxcnt = cnt

		x,y,w,h = cv.boundingRect(maxcnt)
		cv.rectangle(img3,(x,y),(x+w,y+h),(0,0,255),2)
		cv.rectangle(limg,(x,y),(x+w,y+h),(0,0,255),2)
		cv.circle(limg, (x+w/2, y+h/2), 3, (255, 255, 255), thickness=5, lineType=8, shift=0) 
		cv.circle(img3, (x+w/2, y+h/2), 3, (255, 255, 255), thickness=5, lineType=8, shift=0) 

		display_image(img3)
		display_image(limg)

	
		
	#display image
	cv.destroyAllWindows()


if __name__ == "__main__":
    main()
