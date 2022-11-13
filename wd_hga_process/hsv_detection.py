import numpy as np
import cv2, copy

class HSVDetection():
    def __init__(self, color):
        if color == 'pink':
            self.h=[110, 180]
            self.s=[20, 255]
            self.v=[0, 255]
            self.color = [123,82,179]
        elif color == 'orange':
            self.h=[17, 40]
            self.s=[20, 255]
            self.v=[20, 255]
            self.color = [57,179,231]
        elif color == 'red':
            self.h=[0, 10]
            self.s=[100, 255]
            self.v=[100, 255]
            self.color = [57,179,231]
        self.lower_bound = np.array([self.h[0],self.s[0],self.v[0]])
        self.upper_bound = np.array([self.h[1],self.s[1],self.v[1]])
    def process(self, hsv):
        mask = cv2.inRange(copy.deepcopy(hsv), self.lower_bound, self.upper_bound)
        mask = cv2.erode(mask, np.ones((5, 5), dtype=np.uint8))
        mask = cv2.dilate(mask, np.ones((3, 3), dtype=np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        #print(type(contours))
        lContour = []
        maxArea = 0
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            x, y, _w, _h = cv2.boundingRect(contours[i])
            s = min(_w,_h)/max(_w,_h)
            p = float(area) / float(_w*_h)
            if p > 0.55 and s > 0.5:
                if len(lContour) < 2:
                    lContour.append(contours[i])
                elif area > cv2.contourArea(lContour[0]):
                    lContour[0] = contours[i]
                elif area > cv2.contourArea(lContour[1]):
                    lContour[1] = contours[i]
        try:
            x0,y0,_,_ = cv2.boundingRect(lContour[0])
            x1,y1,_,_ = cv2.boundingRect(lContour[1])
            #print('{}, {} : before'.format((x0,y0), (x1,y1)))
        except:
            #print('except : x0,_,_,_ = cv2.boundingRect(lContour[0])')
            return None, contours
        if x1 < x0:
            tmp = copy.deepcopy(lContour[0])
            lContour[0] = copy.deepcopy(lContour[1])
            lContour[1] = copy.deepcopy(tmp)

        #x0,y0,_,_ = cv2.boundingRect(lContour[0])
        #x1,y1,_,_ = cv2.boundingRect(lContour[1])
        #print('{}, {}'.format((x0,y0), (x1,y1)))

        return lContour, contours
    def drawAll(self, _img, contours, debug=False):
        if contours == None:
            return False
        _color = self.color
        if debug:
            _color = [0,0,0]
        for i in range(len(contours)):
            '''
            area = cv2.contourArea(contours[i])
            x, y, _w, _h = cv2.boundingRect(contours[i])
            s = min(_w,_h)/max(_w,_h)
            p = float(area) / float(_w*_h)
            #print(s)
            if p > 0.55 and s > 0.5:
            '''
                #cv2.drawContours(image=_img, contours=contours, contourIdx=i, color=_color, thickness=cv2.FILLED)
            cv2.drawContours(image=_img, contours=contours, contourIdx=i, color=_color, thickness=2)

