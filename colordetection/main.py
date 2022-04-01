# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import cv2 as cv
import numpy as np
import math

def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.


# Press the green button in the gutter to run the script.
sum=np.array([0,0,0]);
if __name__ == '__main__':
    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        # Our operations on the frame come here
        lower_red = np.array([170, 100, 50])
        upper_red = np.array([180, 255, 255])


        test=cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        color = cv.inRange(test,lower_red,upper_red)

        #for index, val in enumerate(color):
        #   print(val)
        #print(color)
        for i in range(len(color)):
            for j in range(len(color[i])):
                if(color[i][j]>=250):
                    sum[0]+=i
                    sum[1]+=j
                    sum[2]+=1
        #print(np.shape(frame))
        #print(np.shape(color))
        #print(sum[0]/sum[2])
        #print(sum[1]/sum[2])
        AVI=math.floor(sum[0]/sum[2])
        AVJ = math.floor(sum[1]/sum[2])
        #print("END \n")

        # Display the resulting frame
        #color = cv.cvtColor(frame, cv.COLOR_HSV2BGR)
        #color[AVI][AVJ]=0
        for i in range(len(color)):
            for j in range(len(color[i])):
                if(i==AVI):
                    color[i][j]=150
                if(j==AVJ):
                    color[i][j] = 150


        cv.imshow('frame', color)
        if cv.waitKey(1) == ord('q'):
            break


# When everything done, release the capture
cap.release()
cv.destroyAllWindows()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
