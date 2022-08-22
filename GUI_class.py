import cv2
import numpy as np
from time import time

from tkinter import *
import tkinter as tk
from PIL import Image, ImageTk

from imutils.video import VideoStream

from shapely.geometry import Point
from shapely.geometry import Polygon

from Detector import Detector

detector  = Detector()

class GUI_Humandetector:

    def __init__(self, cameraURL):
        
        self.setupGUI()

        # Parameter
        self.polyPoints = []
        self.getPoints = False
        self.drawEnable = False
        self.detectEnable = False
        self.start_time = 0

        # Setup camera
        self.cameraURL = cameraURL
        self.cap = VideoStream(self.cameraURL).start()

        # Start
        self.Process()
        self.Window.mainloop()

    def setupGUI(self):
        ##### CREATE WINDOW #######################
        self.Window = tk.Tk()                    ##
        self.Window.geometry("640x600")          ##
        self.Window.title("Window")              ##
        ###########################################

        ################## Label #####################################################
        self.displayFrame = tk.Label(self.Window)                                   ##
        self.displayFrame.place(x=0, y=0)                                           ##
        self.displayFrame.bind('<Button-1>', self.leftButton_pressed)               ##
        self.fpsLabel = Label(self.Window, text="FPS = ")                           ##
        self.fpsLabel.place(x=0, y=520)                                             ##
        self.messageLabel = Label(self.Window, text="Message")                      ##
        self.messageLabel.place(x=0, y=550) 
        Label(self.Window, text="nothin", )                                     ##
        ##############################################################################

        ###################### Create Button #################################################################################
        self.startButton = tk.Button(self.Window, text="Start Drawing", command=self.startButton_Function)                  ##
        self.startButton.place(x=50, y=490)                                                                                 ##
        self.drawButton = tk.Button(self.Window, text="Draw", command= self.drawButton_Function, state= DISABLED)           ##
        self.drawButton.place(x=150, y=490)                                                                                 ##
        self.clearButton = tk.Button(self.Window, text="Clear", command= self.clearButton_Function)                         ##
        self.clearButton.place(x=200, y=490)                                                                                ##
        self.detectButton = tk.Button(self.Window, text="Detect", command= self.detectButton_Function, state=tk.DISABLED)   ##
        self.detectButton.place(x=250, y=490)                                                                               ##
        self.exitButton = tk.Button(self.Window, text="Exit", command= self.Window.destroy)                                 ##
        self.exitButton.place(x=500, y=500)                                                                                 ##
        ###################### Create Button #################################################################################
      
    ## Function
    def startButton_Function(self):
        if self.drawEnable == False:
            self.getPoints = True

    def drawButton_Function(self):
        self.drawButton["state"] = tk.DISABLED
        self.getPoints = False
        self.drawEnable = True
        self.detectButton["state"] = tk.NORMAL
        pass

    def clearButton_Function(self):
        self.getPoints = False
        self.polyPoints = []
        self.drawEnable = False
        self.detectEnable = False

    def detectButton_Function(self):
        self.detectEnable = True

    def leftButton_pressed(self, event):
        if self.getPoints:
            self.polyPoints.append([event.x, event.y])

        if len(self.polyPoints) >= 3 and self.getPoints:
            self.drawButton["state"] = tk.NORMAL
        elif len(self.polyPoints) < 3:
            self.drawButton["state"] = tk.DISABLED

    def drawPolygon (self, frame, points):
        for point in points:
            frame = cv2.circle(frame, (point[0], point[1]), 5, (255,0,0), -1)

        frame = cv2.polylines(frame, [np.int32(points)], isClosed=self.drawEnable, color=(0, 255, 0), thickness=2)
        return frame

    def findLimitPoints(self, points):
        xmin, ymin, xmax, ymax = points[0][0], points[0][1], 0, 0
        for point in points:
            if point[0] < xmin:
                xmin = point[0]
            if point[0] > xmax:
                xmax = point[0]
            if point[1] < ymin:
                ymin = point[1]
            if point[1] > ymax:
                ymax = point[1]

        xmin, ymin, xmax, ymax = xmin-80, ymin-80, xmax+80, ymax+80
        if xmin < 0:
            xmin = 0
        if ymin < 0:
            ymin = 0
        if xmax > 640:
            xmax = 640
        if ymax > 480:
            ymax = 480
        return xmin, ymin, xmax, ymax

    def inSide(self, PolyPoints, detectPoints):

        x1,x2,y1,y2 = detectPoints[0], detectPoints[1], detectPoints[2], detectPoints[3]
        polygon = Polygon(PolyPoints)

        #centroid
        x_centroid = int((x1+x2)/2)
        y_centroid = int((y1+y2)/2)
        centroid = Point(x_centroid, y_centroid)

        #Another Points
        leftTop = Point(int((x1+x_centroid)/2), int((y1+y_centroid)/2))
        rightTop = Point(int((x2+x_centroid)/2), int((y1+y_centroid)/2))
        leftBottom = Point(int((x1+x_centroid)/2), int((y2+y_centroid)/2))
        rightBottom  = Point(int((x2+x_centroid)/2), int((y2+y_centroid)/2))
        
        #Get the boolean
        centroid_result = polygon.contains(centroid)
        leftTop_result = polygon.contains(leftTop)
        rightTop_result = polygon.contains(rightTop)
        leftBottom_result = polygon.contains(leftBottom)
        rightBottom_result = polygon.contains(rightBottom)

        result = centroid_result or ((leftTop_result and rightTop_result) or
                                    (leftTop_result and leftBottom_result) or
                                    (rightTop_result and rightBottom_result) or
                                    (leftBottom_result and rightBottom_result))

        return result

    def showFrame(self, originalFrame):
        finalFrame = cv2.resize(originalFrame, (640,480))
        finalFrame = cv2.cvtColor(finalFrame, cv2.COLOR_BGR2RGB)
        finalFrame_Image = Image.fromarray(finalFrame)
        finalFrame_Image_tk = ImageTk.PhotoImage(image = finalFrame_Image)
        self.displayFrame.imgtk = finalFrame_Image_tk
        self.displayFrame.configure(image = finalFrame_Image_tk)
        self.displayFrame.after(10, self.Process)

    def Process(self):
        self.current_time = time()
        if self.current_time == self.start_time:
            fps = 30
        else:
            fps = int(1/(self.current_time-self.start_time))
        self.fpsLabel["text"] = "FPS = " + str(fps)
        self.start_time = self.current_time

        #Get the Image from camera and resize Image
        originalFrame = self.cap.read()
        originalFrame = cv2.flip(originalFrame, 1)
        originalFrame = cv2.resize(originalFrame, (640,480))

        #Draw the line on the Image
        drawedFrame = self.drawPolygon(originalFrame, self.polyPoints)

        finalFrame = drawedFrame

        self.messageLabel["text"] = "No human"
        self.messageLabel["bg"] = "lime"
        #Image Process
        if self.detectEnable == True:
            # Get the limit of Detect Region
            x1, y1, x2, y2 = self.findLimitPoints(self.polyPoints)

            # Get the frame to detect
            detectFrame = drawedFrame[y1:y2, x1:x2, :]

            # Get the detected points
            detectedPoints = detector.detectBoundingBox(detectFrame, threshold=0.5)

            if len(detectedPoints) > 0: # Deteted human in frame
                polyPoints_calib = []
                for point in self.polyPoints:
                    x_calib = point[0] - x1
                    y_calib = point[1] - y1
                    polyPoints_calib.append([x_calib, y_calib]) # Get the position of pylogon in detect frame
                
                for point in detectedPoints:
                    detect_human = self.inSide(polyPoints_calib, point)
                    if detect_human == True:
                        cv2.rectangle(detectFrame, (point[0],point[2]), (point[1],point[3]), color=(0,0,255), thickness=1)
                        self.messageLabel["text"] = "Human detected!!"
                        self.messageLabel["bg"] = 'red'
                    
                    finalFrame = drawedFrame
                
            finalFrame[y1:y2, x1:x2, :] = detectFrame

        self.showFrame(finalFrame)

    

        

        
        

        
