import RPi.GPIO as GPIO
from picamera2 import PiCamera, Preview
from time import sleep, strftime
from pathlib import Path

camera = PiCamera()
camera. resolution = (1280, 720)

#Static image settings
camera.iso = 100
camera.shutter_speed = 3162 
camera.exposure_mode = 'off' 
camera.awb_mode = 'off' 
camera.awb_gains = (1.1, 2.07)
filepath = Path('/home/siltumnica/Pics')
GPIO. setmode(GPIO.BCM)
GPIO.setup(4, GPIO.OUT) 
GPIO.output(4, False)

def unique_path():
    index = 0
    date_now = strftime("%Y%m%d-%H%M%S")
    filename = f"{date_now}_{index}.jpg"
    shortFilename = f"{date_now}.jpg"
    if not (filepath / shortFilename ).resolve().exists():
        return (filepath / shortFilename).resolve()
    else:
        while (filepath / filename).resolve().exists():
            filename = f"picture_{index}.jpg"
            index += 1
        return (filepath / filename).resolve()

def takePic():
    GPIO.output(4,True)
    camera.start_preview()
    sleep(2)
    camera.capture(str(unique_path()))
    camera.stop_preview()
    GPIO.output(4,False)
    camera.close

while True:
    takePic()
    sleep(1000)
