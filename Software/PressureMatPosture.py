import numpy as np
import matplotlib.pyplot as plt
import serial
import sys
import time
import cv2
import subprocess

CONTOUR = True

if CONTOUR:
    plt.style.use('_mpl-gallery-nogrid')

# This connects to serial

ser = serial.Serial(
    #You might change the PORT corresponding to the assigned by Operative System
     '/dev/cu.usbmodem101269201',  # raspberry: '/dev/ttyUSB1'
    #getPort(),
    baudrate=115200,
    timeout=0.1)


# Default parameters
ROWS = 48  # Rows of the sensor
COLS = 48  # Columns of the sensor

# Variable declaration
# Initialize empty matrix, you ninny (above)
Values = np.zeros((ROWS, COLS))

def getPort():
    print("Getting port")
    PORT = subprocess.run(['ls', '/dev/cu*'], stdout=subprocess.PIPE)
    PORT.stdout
    #return PORT

def RequestPressureMap():
    data = "R"
    ser.write(data.encode())


def activePointsReceiveMap():
    global Values
    matrix = np.zeros((ROWS, COLS), dtype=int)

    xbyte = ser.read().decode('utf-8')

    HighByte = ser.read()
    LowByte = ser.read()
    high = int.from_bytes(HighByte, 'big')
    low = int.from_bytes(LowByte, 'big')
    nPoints = ((high << 8) | low)

    xbyte = ser.read().decode('utf-8')
    xbyte = ser.read().decode('utf-8')
    x = 0
    y = 0
    n = 0
    while(n < nPoints):
        x = ser.read()
        y = ser.read()
        x = int.from_bytes(x, 'big')
        y = int.from_bytes(y, 'big')
        HighByte = ser.read()
        LowByte = ser.read()
        high = int.from_bytes(HighByte, 'big')
        low = int.from_bytes(LowByte, 'big')
        val = ((high << 8) | low)
        matrix[y][x] = val
        n += 1
    Values = matrix
 
def activePointsGetMap():
    xbyte = ''
    if ser.in_waiting > 0:
        try:
            xbyte = ser.read().decode('utf-8')
        except Exception:
            print("Exception")
        if(xbyte == 'N'):
            activePointsReceiveMap()
        else:
            ser.flush()

class Null:
    def write(self, text):
        pass

    def flush(self):
        pass

def getMatrix():
    RequestPressureMap()
    activePointsGetMap()

def printMatrix():
    tmparray = np.zeros((ROWS, COLS))
    for i in range(COLS):
        tmp = ""
        for j in range(ROWS):
            tmp = int(Values[i][j])
            tmparray[i][j] = tmp
    if CONTOUR:
        generatePlot(tmparray)
    print("\n")
    for i in range(COLS):
        tmp = ""
        for j in range(ROWS):
            tmp = tmp +   hex(int(Values[i][j]))[-1]
        print(tmp)
    print("\n")

def generatePlot(Z):
    plt.ion()
    fig, ax = plt.subplots(figsize=(5,5))

    ax.contourf(np.arange(0, ROWS), np.arange(0, COLS), Z, levels=7, cmap="nipy_spectral")

    plt.draw()
    plt.savefig('./Results/contour.png')
    plt.pause(0.0001)
    plt.clf()

def printArray(A):
    for item in A:
        print(item, sep="", end="")

def getBlobs():
    # Read image
    im = cv2.imread("./Results/contour.png", cv2.IMREAD_GRAYSCALE)

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 10
    params.maxThreshold = 200


    # Filter by Area.
    params.filterByArea = True
    params.minArea = 1500

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.87
        
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.01

    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3 :
        detector = cv2.SimpleBlobDetector(params)
    else : 
        detector = cv2.SimpleBlobDetector_create(params)


    # Detect blobs.
    keypoints = detector.detect(im)
    print("keypoints")
    print(keypoints)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
    # the size of the circle corresponds to the size of blob
    if(keypoints):
        im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Show blobs
        cv2.imshow("Keypoints", im_with_keypoints)
        cv2.waitKey(0)

def getBlobs2():
    # read image through command line
    img = cv2.imread("./Results/contour.png")

    # convert the image to grayscale
    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # cv2.imshow("Grayscale", gray_image)
    # cv2.waitKey(0)

    # convert the grayscale image to binary image
    ret, thresh = cv2.threshold(gray_image,127,255,0)

    # find contours in the binary image
    # im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    im2, contours = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        # calculate moments for each contour
        M = cv2.moments(c)

        # calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
        cv2.putText(img, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # display the image
        cv2.imshow("Image", img)
        cv2.waitKey(0)
   
#Main

# if __name__ == 'main':

while True:
    pass
    getMatrix() # This function requests and parses a pressure map in the variable Values    
    printMatrix()
    getBlobs()
    # time.sleep(1)

