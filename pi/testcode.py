import numpy as np
import cv2 as cv
import cv2.cv as cv2



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
	display_image(r)
	g = img[:, :, 1]
	g = cv.inRange(g, gmin, gmax)
	display_image(g)
	b = img[:, :, 0]
	b = cv.inRange(b, bmin, bmax)
	display_image(b)
	img2 = cv.merge((b, g, r))
	display_image(img2)
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


def main():
	for i in range(1, 8):
		#load image
		limg = load_image("test_pics/lv_" + str(i) + ".jpg");
		raw_img = limg.copy()
		display_image(raw_img)
		#gaussian blur
		raw_img = gaussian_blur(raw_img, 3)
		display_image(raw_img)

		raw_img = range_threshold_color(raw_img, 0, 30, 20, 80, 50, 125)
		display_image(raw_img)

		#grayscale
		raw_img = grayscale(raw_img)
		display_image(raw_img)

		#threshold
		raw_img = threshold(raw_img, 254, 255, cv.THRESH_BINARY)
		display_image(raw_img)

		raw_img = gaussian_blur(raw_img, 11)
		display_image(raw_img)

		#threshold
		raw_img = threshold(raw_img, 20, 255, cv.THRESH_BINARY)
		display_image(raw_img)

		#canny edge detect
		raw_img = canny(raw_img, 100, 100);
		display_image(raw_img)
		
		raw_img = gaussian_blur(raw_img, 3)
		display_image(raw_img)   

		#hough lines
		raw_img, raw_img2 = hough_lines(raw_img, limg, 20, 1000, 50)
		display_image(raw_img2)
		display_image(raw_img)
	#display image
	cv.destroyAllWindows()


if __name__ == "__main__":
    main()
