import cv2, copy
import numpy as np

path = 'C:/wd_hga_process/'
fname = '1243175.jpg'
img = None
hsv_debug = False

class HSVDetection():
    def __init__(self, color):
        if color == 'pink':
            self.h=[160, 175]
            self.s=[20, 255]
            self.v=[20, 255]
            self.color = [123,82,179]
        elif color == 'orange':
            self.h=[17, 40]
            self.s=[20, 255]
            self.v=[20, 255]
            self.color = [57,179,231]
        self.lower_bound = np.array([self.h[0],self.s[0],self.v[0]])
        self.upper_bound = np.array([self.h[1],self.s[1],self.v[1]])
    def process(self, hsv):
        mask = cv2.inRange(copy.deepcopy(hsv), self.lower_bound, self.upper_bound)
        mask = cv2.erode(mask, np.ones((5, 5), dtype=np.uint8))
        mask = cv2.dilate(mask, np.ones((3, 3), dtype=np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        return contours
    def drawAll(self, _img, contours, debug=False):
        _color = self.color
        if debug:
            _color = [0,0,0]
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            x, y, _w, _h = cv2.boundingRect(contours[i])
            s = min(_w,_h)/max(_w,_h)
            p = float(area) / float(_w*_h)
            print(s)
            if p > 0.75 and s > 0.75:
                #cv2.drawContours(image=_img, contours=contours, contourIdx=i, color=_color, thickness=cv2.FILLED)
                cv2.drawContours(image=_img, contours=contours, contourIdx=i, color=_color, thickness=2)
        

def main():
    global img

    img = cv2.imread(path+fname)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = cv2.blur(hsv, (7,7))
    
    orange = HSVDetection('orange')
    contours = orange.process(hsv)
    orange.drawAll(img, contours, hsv_debug)

    pink = HSVDetection('pink')
    contours = pink.process(hsv)
    pink.drawAll(img, contours, hsv_debug)

    #img = hsv
    
    # Displaying the image
    cv2.imshow('image', img)
    cv2.setMouseCallback('image', click_event)
    cv2.waitKey(0)

def click_event(event, x, y, flags, params):
    global img
 
    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
 
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)
 
        # displaying the coordinates
        # on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, str(x) + ',' +
                    str(y), (x,y), font,
                    1, (255, 0, 0), 2)
        cv2.imshow('image', img)
 
    # checking for right mouse clicks    
    if event==cv2.EVENT_RBUTTONDOWN:
 
        # displaying the coordinates
        # on the Shell
        print(x, ' ', y)
 
        # displaying the coordinates
        # on the image window
        font = cv2.FONT_HERSHEY_SIMPLEX
        b = img[y, x, 0]
        g = img[y, x, 1]
        r = img[y, x, 2]
        print(type(b))
        cv2.putText(img, str(b) + ',' +
                    str(g) + ',' + str(r),
                    (x,y), font, 1,
                    (int(b), int(g), int(r)), 2)
        cv2.imshow('image', img)
        
if __name__ == '__main__':
    main()
