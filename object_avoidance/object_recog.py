import cv2
import cv2.cv as cv
import numpy as np
import math, operator

# calculates cosines between three points
def angle_cos(p0, p1, p2):
    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

def find_squares(img):
    img = cv2.GaussianBlur(img, (5, 5), 0)
    squares = []
    centers = []

    count = 0
    sigma=0.33
    # compute the median of the single channel pixel intensities
    v = np.median(img)
 
    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))

    # split image, take any one of the channels
    for gray in cv2.split(img):
    	# for now, only do a one iteration of the loop
        for thrs in xrange(0, 255, 255):
            if thrs == 0:
            	# canny edge detection -- http://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html
                bin = cv2.Canny(gray, lower, upper)
                # dilate -- do not dilate for now
                bin = cv2.dilate(bin, None)
            else:
                retval, bin = cv2.threshold(gray, thrs, 255, cv2.THRESH_BINARY)

            # find contours in the image
            contours, hierarchy = cv2.findContours(bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            # loop through each contour
            for cnt in contours:

            	# approximate a polynomial figure
                cnt_len = cv2.arcLength(cnt, True)
                cnt = cv2.approxPolyDP(cnt, 0.05*cnt_len, True)

                # check if it meets minimum requirements -- TODO
                if len(cnt) > 2 and cv2.contourArea(cnt) > 600 and cv2.isContourConvex(cnt):
                    (x, y, w, h) = cv2.boundingRect(cnt)
                    roi = cv2.boundingRect(cnt)
                    cnt = cnt.reshape(-1, 2)

                    # add the the list of good contours
                    squares.append(cnt)

    return squares

if __name__ == '__main__':
	# load, show image
	im = cv2.imread('water-on-mars.png', 0)
	cv2.imshow('image',im)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

	squares = find_squares(im)

	# draw contours on the image
	cv2.drawContours( im, squares, -1, (255, 0, 0), 3 )
	cv2.imshow('squares', im)
	ch = 0xFF & cv2.waitKey()
	cv2.destroyAllWindows()



