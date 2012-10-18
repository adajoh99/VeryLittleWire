# File: littleWire.py
# Version: v1.0
# Purpose: Provides a Python interface to the Little Wire USB Multi-Tool
#          developed by Ihsan Kehribar
# Author: Adam Johnson
# Copyright 2012 by Adam Johnson <apjohnson@gmail.com>
#
# This file is a direct Python translation of the C++ library by Ihsan Kehribar
# <ihsan@kehribar.me> and Omer Kilic <omerkilic@gmail.com>, version 0.9.
# It is released under the following license, the same as the original C++ source.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
# of the Software, and to permit persons to whom the Software is furnished to do
# so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import usb.core
import usb.control

# Constants--------------------------------------------------------------------

#USB constants
VENDOR_ID = 0x1781
PRODUCT_ID = 0x0C9F
USB_TIMEOUT = 5000
RX_BUFFER_SIZE = 64

#Pin Definitions

#General Purpose Pins
PIN1 = 1
PIN2 = 2
PIN3 = 5
PIN4 = 0

#ADC Channels
ADC_PIN3 = 0
ADC_PIN2 = 1
ADC_TEMP_SENS = 2

#PWM Pins
PWM1 = PIN4
PWM2 = PIN1

#Aliases
ADC0 = ADC_PIN3
ADC1 = ADC_PIN2
ADC2 = ADC_TEMP_SENS
PWMA = PWM1
PWMB = PWM2

#'AVR ISP' Pins
SCK_PIN = PIN2
MISO_PIN = PIN1
MOSI_PIN = PIN4
RESET_PIN = PIN3

#Servo constants
STEP_SIZE = 0.062   #in miliseconds
MIN_LIMIT = 0.8     #in miliseconds
MAX_LIMIT = 2.2     #in miliseconds
RANGE = 180         #in degrees

#Useful constants

INPUT = 1
OUTPUT = 0

AUTO_CS = 1
MANUAL_CS = 0

HIGH = 1
LOW = 0

# Code ------------------------------------------------------------------------

class device:
    """ Class to control a LittleWire USB Multi-Tool."""
    lw = None

    def __init__(self):
        """Finds the first littleWire and attaches to it"""
        self.lw = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
        
        if self.lw == None:
            raise ValueError('Device not Found!')
        
        self.lw.set_configuration()
        
    def readFirmwareVersion(self):
        """Returns littleWire firmware version"""
        rtn = self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=34,
                                    wValue=0, wIndex=0,
                                    data_or_wLength=8, timeout=USB_TIMEOUT)
        version = rtn.pop()
        return str((version & 0xF0) >> 4 )+"." + str((version & 0x0F))
    
    def pinMode(self, pin, mode):
        """Sets GPIO pins to INPUT(1)or OUTPUT(0)"""
        if (mode == INPUT):
            self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=13,
                                  wValue=pin, wIndex=0,
                                  data_or_wLength=8, timeout=USB_TIMEOUT)
        else:
            self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=14,
                                  wValue=pin, wIndex=0,
                                  data_or_wLength=8, timeout=USB_TIMEOUT)
    
    def digitalWrite(self, pin, state):
        """Writes a digital HIGH (1) or LOW (0) to the selected GPIO"""
        if (state == 1):
            self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=18,
                                  wValue=pin, wIndex=0,
                                  data_or_wLength=8, timeout=USB_TIMEOUT)
        else:
            self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=19,
                                  wValue=pin, wIndex=0,
                                  data_or_wLength=8, timeout=USB_TIMEOUT)
        
    def digitalRead(self, pin):
        """Returns the digital status of the selected GPIO"""
        rtn = self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=20,
                                    wValue=pin, wIndex=0,
                                    data_or_wLength=8, timeout=USB_TIMEOUT)
        return rtn.pop()
        
    def analogread(self, channel):
        """Returns the analog value of the selected analog pin"""
        rtn = self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=15,
                                    wValue=channel, wIndex=0,
                                    data_or_wLength=8, timeout=USB_TIMEOUT)
        return rtn.pop()
    
    def pwm_init(self):
        """Initialises the PWM system.  Must be called before using PWM commands"""
        self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=16,
                              wValue=0, wIndex=0,
                              data_or_wLength=8, timeout=USB_TIMEOUT)
        
    def pwm_stop(self):
        """Stops the PWM outputs"""
        self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=32,
                              wValue=0, wIndex=0,
                              data_or_wLength=8, timeout=USB_TIMEOUT)

    def pwm_updateCompare(self, channelA=0, channelB=0):
        """Sets the PWM value for both channels"""
        self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=32,
                              wValue=channelA, wIndex=channelB,
                              data_or_wLength=8, timeout=USB_TIMEOUT)

    def pwm_updatePrescaler(self, value=1):
        """Sets the value of the PWM prescaler - controls the PWM frequency"""
        if (value == 1024):
            self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=22,
                                  wValue=4, wIndex=0,
                                  data_or_wLength=8, timeout=USB_TIMEOUT)
        elif (value == 256):
            self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=22,
                                  wValue=3, wIndex=0,
                                  data_or_wLength=8, timeout=USB_TIMEOUT)
        elif (value == 64):
            self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=22,
                                  wValue=2, wIndex=0,
                                  data_or_wLength=8, timeout=USB_TIMEOUT)
        elif (value == 8):
            self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=22,
                                  wValue=1, wIndex=0,
                                  data_or_wLength=8, timeout=USB_TIMEOUT)
        elif (value == 1):
            self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=22,
                                  wValue=0, wIndex=0,
                                  data_or_wLength=8, timeout=USB_TIMEOUT)
                
    def spi_init(self):
        """Initialises the SPI comms system"""
        self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=23,
                              wValue=0, wIndex=0,
                              data_or_wLength=8, timeout=USB_TIMEOUT)
        
    def spi_sendMessage(self, message):
        """Sends a message out on the SPI bus"""
        self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=21,
                              wValue=message, wIndex=0,
                              data_or_wLength=8, timeout=USB_TIMEOUT)

    def spi_sendMessageMulti(self, sendBuffer, length, mode):
        """Sends up to 4 bytes out on the SPI bus and reads the same number back"""
        if (length > 4):
            length = 4

        rtn = self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=(0xF0 + length + (mode <<3)),
                              wValue=(sendbuffer[1]<<8)+sendbuffer[0],
                              wIndex=(sendbuffer[3]<<8)+sendbuffer[2],
                              data_or_wLength=8, timeout=USB_TIMEOUT)
        return rtn
    
    def spi_updateDelay(self, duration):
        self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=31,
                              wValue=duration, wIndex=0,
                              data_or_wLength=8, timeout=USB_TIMEOUT)      

    def i2c_init(self):
        """Initialises the I2C comms system"""
        self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=24,
                              wValue=0, wIndex=0,
                              data_or_wLength=8, timeout=USB_TIMEOUT)
        
    def i2c_beginTransmission(self, address):
        self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=25,
                              wValue=address, wIndex=0,
                              data_or_wLength=8, timeout=USB_TIMEOUT)

    def i2c_send(self, message):
        self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=26,
                              wValue=message, wIndex=0,
                              data_or_wLength=8, timeout=USB_TIMEOUT)
        
    def i2c_endTransmission(self):
        self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=27,
                              wValue=0, wIndex=0,
                              data_or_wLength=8, timeout=USB_TIMEOUT)
        
    def i2c_requestFrom(self, address, numBytes, responseBuffer):
        rtn = self.lw.ctrl_transfer(bmRequestType=0xC0, bRequest=30,
                              wValue=address, wIndex=numBytes,
                              data_or_wLength=8, timeout=USB_TIMEOUT)
        return rtn

    def servo_init(self):
        """Initialises the Servo control"""
        self.pwm_init() #initialise the PWM hardware
        self.pinMode(PWMA, OUTPUT)
        self.pinMode(PWMB, OUTPUT)  #set the PWM pins output
        self.pwm_updatePrescaler(1024)  #make sure the PWM prescaler is set correctly

    def servo_updateLocation(self, locationChannelA, locationChannelB):
        """Moves servos to a new location"""
        locationChannelA = int((((locationChannelA/RANGE)*(MAX_LIMIT-MIN_LIMIT))+MIN_LIMIT)/STEP_SIZE)
        locationChannelB = int((((locationChannelB/RANGE)*(MAX_LIMIT-MIN_LIMIT))+MIN_LIMIT)/STEP_SIZE)
        self.pwm_updateCompare(locationChannelA, locationChannelB)

