# importing the required libraries
from djitellopy import tello
import cv2
import numpy as np
import time
from pyzbar.pyzbar import decode
# initiating connection with the drone
me = tello.Tello()
me.connect()
# starting the video stream
me.streamon()
# taking-off
me.takeoff()
# increasing the elevation a little after taking off (to reach human height)
me.send_rc_control(0,0,25,0)
time.sleep(5)
me.send_rc_control(0,0,0,0)
# some parameters that are required in the code
# width and height of the resized image
w, h = 720, 480
# thresholds for Front/Backward movement (get this through trial and error)
fbRange = [10000 , 13000]
# Parameters of PID controller (Proportional, Derivative, Integral)
pid = [0.4 , 0.4 , 0]
# previous error values (needed to compute the PID gain)
pError = [0 , 0]
def detect_qr(img):
    myFaceListC = []
    myFaceListArea = []
    if not decode(img):
        return img, [[0, 0], 0], ''
    for barcode in decode(img):
        my_data = barcode.data.decode('utf-8')
        print(my_data)
        if not my_data:
            my_data = ''
        x = barcode.rect.left
        y = barcode.rect.top
        w = barcode.rect.width
        h = barcode.rect.height
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        # calculate the area
        area = w * h
        cx = x + w//2
        cy = y + h//2
        #add the center and area to the Faces lists
        myFaceListC.append([cx, cy])
        myFaceListArea.append(area)
        # if statement to check if we detect any faces or not
        if len(myFaceListArea) != 0:
            # if found, return the largest face (largets area)
            i = myFaceListArea.index(max(myFaceListArea))
            return img, [myFaceListC[i], myFaceListArea[i]], my_data
        else:
            # if not found, return zeros
            return img, [[0, 0], 0], ''
# function for sending velocity commands to the drone
def trackFace( me , info , w  , h, pid , pErrorX , pErrorY, cmd):    #me,
    # get the features for driving the control
    area = info[1]
    x,y = info[0]
    fb = 0
    errorY = y - h//2
    errorX = x - w//2
    # calculating the X component speed (yaw command)
    speedX = pid[0]*errorX + pid[1]*(errorX- pErrorX) #PD controller
    speedX = int(np.clip(speedX , -100 , 100))
    # calculating the Y component speed (elevation command)
    speedY = pid[0]*errorY + pid[1]*(errorY- pErrorY) #PD controller
    speedY = int(np.clip(speedY , -30 , 30))
    # calculating the Front/Backward speed
    if area > fbRange[0] and area < fbRange[1]:
        fb = 0
        if cmd != '':
            move(cmd)
    elif area > fbRange[1]:
        fb = -15
    elif  area < fbRange[0] and area !=0 :
        fb = 15
    # stopping the movement when the face is at the image center
    if x == 0:
        speedX =0
        speedY =0
        errorX =0
        errorY =0
    # prining the instantaneous speed command sent to the drone
    print( "speed X:" , speedX , "Speed Y: ", speedY,  "Speed Front-back:" , fb )         # "Error:" , error ,
    # sending the speed command to the drone
    me.send_rc_control(0,fb,-speedY,speedX)
    # returning the current errors for the next iteration
    error = [errorX, errorY]
    return error
def move(cmd):
    if not cmd:
        return
    if cmd == 'right':
        me.rotate_clockwise(90)
        me.move_forward(30)
    elif cmd == 'left':
        me.rotate_counter_clockwise(90)
        me.move_forward(30)
    elif cmd == 'land':
        me.land()
    else:
        print(f'{cmd} was not done')
    return
# loop to keep getting frames and controlling the drone
time.sleep(2)
while True:
    # get a single frame from the drone's camera
    img = me.get_frame_read().frame
    # resize the frame
    img = cv2.resize(img, (w, h))
    # get face detections using the function findFace()
    img , info, cmd = detect_qr(img)
    # call the function trackFace() to generate velocity commands and get current error
    pError = trackFace( me , info, w, h, pid, pError[0], pError[1], cmd)
    # print the center and the area of the detected face
    #print("Center" , info[0] , "Area" , info[1])
    # display the image with the box
    cv2.imshow("Output" , img)
    # if statement for waiting 1 ms and landing if you click "q" on the keyboard
    if cv2.waitKey(1) & 0xFF == ord('q'):
        me.land()
        break
