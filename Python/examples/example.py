import littleWire
import time

#initialise the LittleWire
lw = littleWire.device()

print "Press ctrl-c to exit\n"

#Set pin 1 as a digital OUTPUT
lw.pinMode(littleWire.PIN1, littleWire.OUTPUT)


#generate a 0.5Hz square wave on pin 1 (1 second HIGH, 1 second LOW)
while(1):
    try:
        lw.digitalWrite(littleWire.PIN1, littleWire.HIGH)
        time.sleep(1)
        lw.digitalWrite(littleWire.PIN1, littleWire.LOW)
        time.sleep(1)

    except:
        break

