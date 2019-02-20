import cv2
import imutils
import numpy as np

#func for getting color of click pos
def click_pos(event, x, y, flags, param):
	global mouseX , mouseY
	if event == cv2.EVENT_LBUTTONDOWN:
		mouseX, mouseY = x, y
		print(frame[y,x])


# while True:
	# cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
global frame
frame = cv2.imread("FlightGoggles.png")


redLower = (0, 0, 254)
redUpper = (0, 0, 255)
mask = cv2.inRange(frame, redLower, redUpper)



cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
cv2.resizeWindow('Image', 1024,768)
cv2.imshow("Image", mask)
cv2.setMouseCallback('Image', click_pos)
	
	# cv2.setMouseCallback('frame', click_pos)

cv2.waitKey(0)
cap.release()
cv2.destroyAllWindows()
