from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO     #for GPIO
import spidev               #for cds
import time                 
import cv2                  #for detecting face
import serial

#for transfer to arduino
k = 0
pin = 17

#decting Face
def detect(img, cascade):
    rects = cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=4, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)
    if len(rects) == 0:
        return []
    rects[:,2:] += rects[:,:2]
    return rects

#if finding face, draw rectangle
def draw_rects(img, rects, color):
    for x1,y1,x2,y2 in rects:
        cv2.rectangle(img, (x1,y1), (x2,y2), color, 2)
        global k
        k=1

#opencv main
def FF():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(14,GPIO.OUT)
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        img = frame.array
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        rects = detect(gray, cascade)
        vis = img.copy()
        draw_rects(vis, rects, (0, 255, 0))
        cv2.imshow("Frame", vis)
        key =cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)
        if key == ord("q"):
            break
        #seding to arduino
        global k
        if(k==1):
            GPIO.output(14,GPIO.HIGH)
            k=0
        else :
            GPIO.output(14,GPIO.LOW)
            k=0

#CV2 setup
camera = PiCamera()
camera.resolution = (480, 320)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(480,320))
cascade = cv2.CascadeClassifier("opencv/data/haarcascades/haarcascade_frontalface_alt.xml")
time.sleep(0.1)

#spi setup
spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz = 1000000

FF()

