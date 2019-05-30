# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import dht11
import smbus
from time import sleep

GPIO.setwarnings(False)
GPIO.cleanup()

# Define some device parameters
I2C_ADDR  = 0x27 # I2C device address #アドレスが違う場合もあるかもしれない
LCD_WIDTH = 16   # Maximum characters per line

# Define some device constants
LCD_CHR = 1 # Mode - Sending data
LCD_CMD = 0 # Mode - Sending command

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line

LCD_BACKLIGHT  = 0x08  # On
#LCD_BACKLIGHT = 0x00  # Off

ENABLE = 0b00000100 # Enable bit

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

#Open I2C interface
#bus = smbus.SMBus(0)  # Rev 1 Pi uses 0
bus = smbus.SMBus(1) # Rev 2 Pi uses 1

def lcd_init():
    # Initialise display
    lcd_byte(0x33,LCD_CMD) # 110011 Initialise
    lcd_byte(0x32,LCD_CMD) # 110010 Initialise
    lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
    lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
    lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
    lcd_byte(0x01,LCD_CMD) # 000001 Clear display
    sleep(E_DELAY)

def lcd_byte(bits, mode):
    # Send byte to data pins
    # bits = the data
    # mode = 1 for data
    #        0 for command

    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT

    # High bits
    bus.write_byte(I2C_ADDR, bits_high)
    lcd_toggle_enable(bits_high)

    # Low bits
    bus.write_byte(I2C_ADDR, bits_low)
    lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
    # Toggle enable
    sleep(E_DELAY)
    bus.write_byte(I2C_ADDR, (bits | ENABLE))
    sleep(E_PULSE)
    bus.write_byte(I2C_ADDR,(bits & ~ENABLE))
    sleep(E_DELAY)


def lcd_string(message,line):
    # Send string to display
    message = message.ljust(LCD_WIDTH," ")
    lcd_byte(line, LCD_CMD)
    for i in range(LCD_WIDTH):
        lcd_byte(ord(message[i]),LCD_CHR)

def tmp_get(TMP_PIN):
    instance = dht11.DHT11(pin = TMP_PIN)

    result = instance.read()

    tmpNum = result.temperature
    hmdNum = result.humidity

    thIndex = round(0.81 * tmpNum + 0.01 * hmdNum * (0.99 * tmpNum - 14.3) + 46.3,1)

    return {"tmpNum": tmpNum, "hmdNum": hmdNum, "thIndex": thIndex}

if __name__ == '__main__':
    #Define GPIO to LCD mapping
    ################
    # Correct here #
    ################
    #TMP_PIN = your_select_pin_number
    #RED_PIN = your_select_pin_number
    #BLUE_PIN = your_select_pin_nubmer
    #GREEN_PIN = your_select_pin_number

    #initialize GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RED_PIN,GPIO.OUT)
    GPIO.setup(GREEN_PIN,GPIO.OUT)
    GPIO.setup(BLUE_PIN,GPIO.OUT)

    value = tmp_get(TMP_PIN)

    if value["tmpNum"] != 0 and value["hmdNum"] != 0:

        # green pin
#        if value["thIndex"] < 75:
        if value["thIndex"] <= 70:
            GPIO.output(GREEN_PIN, True)
        else:
            GPIO.output(GREEN_PIN, False)

        # blue pin
#        if value["thIndex"] > 70 and value["thIndex"] < 85:
        if value["thIndex"] > 70 and value["thIndex"] <= 80:
            GPIO.output(BLUE_PIN, True)
        else:
            GPIO.output(BLUE_PIN, False)

        # red pin
        if value["thIndex"] > 80:
            GPIO.output(RED_PIN, True)
        else:
            GPIO.output(RED_PIN, False)

        #Display
        lcd_init()
        
        lcd_string("tmp:"+str(value["tmpNum"])+"C hmd:"+str(value["hmdNum"])+"%", LCD_LINE_1)
        lcd_string("thIndex:"+str(value["thIndex"]), LCD_LINE_2)
        sleep(5)

        # print("気温："+str(value["tmpNum"])+"C\n湿度："+str(value["hmdNum"])+"%\n不快指数："+str(value["thIndex"]))
        
        LCD_BACKLIGHT = 0x00  #バックライトオフ
        lcd_byte(0x01, LCD_CMD) #表示内容クリア
        GPIO.cleanup()
