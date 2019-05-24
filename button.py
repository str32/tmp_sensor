import RPi.GPIO as GPIO
import time
import signal
import sys
import subprocess

before = 0
sendFlg = 0
offCount = 0
cmdTH = "sudo python slider/tmp_sensor.py"
cmdSD = "sudo python slider/tmp_post.py"

BTN_PIN = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(BTN_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)

def handler(signum, frame):
    print 'Signal handler called with signal', signum
    GPIO.cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, handler)

while True:
    now = GPIO.input(BTN_PIN)
    
    if sendFlg == 1:
        if before == 1:
            if now == 1:
                offCount += 1
            else:
                subprocess.call(cmdTH.split())
                sendFlg = 1
            
        if offCount == 30:
            subprocess.call(cmdSD.split())
            sendFlg = 1
    else:
        if before == 0:
            offCount = 0
            sendFlg = 0

    time.sleep(0.1)
    before = now
    print(str(sendFlg)+'/'+str(before))
