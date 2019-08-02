import RPi.GPIO as GPIO
import time
import subprocess

GPIO.setmode(GPIO.BOARD)

# GPIO port numbers are found in Raspberry Pi specs
RIGHTOUTPUT1 = 22
RIGHTOUTPUT2 = 18
LEFTOUTPUT1 = 36
LEFTOUTPUT2 = 32
IRINPUT1 = 12
IRINPUT2 = 16
TRIG = 37
ECHO = 33
SLEEPTIME = 1

GPIO.setup(LEFTOUTPUT1,GPIO.OUT) #Left motor input A
GPIO.setup(LEFTOUTPUT2,GPIO.OUT) #Left motor input B
GPIO.setup(RIGHTOUTPUT1,GPIO.OUT) #Right motor input A
GPIO.setup(RIGHTOUTPUT2,GPIO.OUT) #Right motor input B

GPIO.setup(IRINPUT1,GPIO.IN)
GPIO.setup(IRINPUT2,GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

GPIO.setwarnings(False)

def stop():
    print "stop"
    GPIO.output(LEFTOUTPUT1, 0)
    GPIO.output(LEFTOUTPUT2, 0)
    GPIO.output(RIGHTOUTPUT1, 0)
    GPIO.output(RIGHTOUTPUT2, 0)

def forward():
    GPIO.output(LEFTOUTPUT1, 0)
    GPIO.output(LEFTOUTPUT2, 1)
    GPIO.output(RIGHTOUTPUT1, 1)
    GPIO.output(RIGHTOUTPUT2, 0)
    print "Forward"
    #execfile("Adafruit_Python_DHT/examples/simpletest.py")

def back():
    GPIO.output(LEFTOUTPUT1, 1)
    GPIO.output(LEFTOUTPUT2, 0)
    GPIO.output(RIGHTOUTPUT1, 0)
    GPIO.output(RIGHTOUTPUT2, 1)
    print "back"
    measureBack()

def left():
    GPIO.output(LEFTOUTPUT1, 0)
    GPIO.output(LEFTOUTPUT2, 0)
    GPIO.output(RIGHTOUTPUT1, 1)
    GPIO.output(RIGHTOUTPUT2, 0)
    print "left"

def right():
    GPIO.output(RIGHTOUTPUT1, 0)
    GPIO.output(RIGHTOUTPUT2, 0)
    GPIO.output(LEFTOUTPUT1, 0)
    GPIO.output(LEFTOUTPUT2, 1)
    print "right"

def measureDistance():
    
    print "Distance measurement in progress"
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    pulse_start = 0
    pulse_end = 0

    while GPIO.input(ECHO)==0:
        pulse_start = time.time()
    while GPIO.input(ECHO)==1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    print "Distance: ",distance," cm"
    return distance

def measureBack():
    i=GPIO.input(IRINPUT1)
    j=GPIO.input(IRINPUT2)
    if i==0 or j==0:
        stop()
        print "Object detected on back"
        time.sleep(1)

# Starts livestreaming from camera
subprocess.Popen(["bash", "RPi_Cam_Web_Interface/start.sh"])
forward()

while True:
    distance = measureDistance()
    time.sleep(0.1)

    GPIO.output(TRIG, False)
    print "Waiting for sensor to settle"
    time.sleep(0.0000001)

    flag=0

    # Check whether distance to any obstacle in front is within 25 cm
    if distance < 25.0:
        stop()
        time.sleep(1)
        back()
        time.sleep(1.5)

        if flag==0:
            time.sleep(0.5)
            right()
            flag=1
        else:
            time.sleep(0.5)
            left()
            flag=0
        time.sleep(1.5)
        stop()
        time.sleep(1)

    else:
        forward()
        flag=0