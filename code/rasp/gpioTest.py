#!/usr/bin/env python3
import time
import RPi.GPIO as GPIO



PIN1 = 1
#PIN2 = 2


def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN1, GPIO.OUT)
    #GPIO.setup(PIN2, GPIO.OUT)
    return



if __name__ == "__main__":

    setup()

    while True:

        GPIO.output(PIN1, GPIO.HIGH)
        print("High on "+ str(PIN1))
        time.sleep(1)
        
        GPIO.output(PIN1, GPIO.LOW)
        print("Low on "+ str(PIN1))
        time.sleep(1)

    GPIO.cleanup()

    


