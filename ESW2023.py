import time
import numpy as np
import cv2

#from debuging_utill import *
from golfESW_front2 import *
from Drive_motors import *

if __name__ == '__main__':
    #debuging_utill()
    time.sleep(0.1)
    move_motor(31)
    time.sleep(0.1)
    golfESW()
