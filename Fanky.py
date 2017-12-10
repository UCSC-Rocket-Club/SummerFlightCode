# Simple demo of of the ADXL345 accelerometer library.  Will print the X, Y, Z
# axis acceleration values every half second.
# Author: Tony DiCola
# License: Public Domain
import time

# Import the ADXL345 module.
# Acceleramoeter
import Adafruit_ADXL345


# Create an ADXL345 instance.
accel = Adafruit_ADXL345.ADXL345()

# Alternatively you can specify the device address and I2C bus with parameters:
#accel = Adafruit_ADXL345.ADXL345(address=0x54, busnum=2)

# You can optionally change the range to one of:
#  - ADXL345_RANGE_2_G   = +/-2G (default)
#  - ADXL345_RANGE_4_G   = +/-4G
#  - ADXL345_RANGE_8_G   = +/-8G
#  - ADXL345_RANGE_16_G  = +/-16G
# For example to set to +/- 16G:
accel.set_range(Adafruit_ADXL345.ADXL345_RANGE_16_G)

# Or change the data rate to one of:
#  - ADXL345_DATARATE_0_10_HZ = 0.1 hz
#  - ADXL345_DATARATE_0_20_HZ = 0.2 hz
#  - ADXL345_DATARATE_0_39_HZ = 0.39 hz
#  - ADXL345_DATARATE_0_78_HZ = 0.78 hz
#  - ADXL345_DATARATE_1_56_HZ = 1.56 hz
#  - ADXL345_DATARATE_3_13_HZ = 3.13 hz
#  - ADXL345_DATARATE_6_25HZ  = 6.25 hz
#  - ADXL345_DATARATE_12_5_HZ = 12.5 hz
#  - ADXL345_DATARATE_25_HZ   = 25 hz
#  - ADXL345_DATARATE_50_HZ   = 50 hz
#  - ADXL345_DATARATE_100_HZ  = 100 hz (default)
#  - ADXL345_DATARATE_200_HZ  = 200 hz
#  - ADXL345_DATARATE_400_HZ  = 400 hz
#  - ADXL345_DATARATE_800_HZ  = 800 hz
#  - ADXL345_DATARATE_1600_HZ = 1600 hz
#  - ADXL345_DATARATE_3200_HZ = 3200 hz
# For example to set to 6.25 hz:
accel.set_data_rate(Adafruit_ADXL345.ADXL345_DATARATE_100_HZ)


#########################################################

import timeit

import smbus

bus = smbus.SMBus(1)

index=0

import numpy as np


#########################################################


#########################################################


from Tkinter import *
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
pwm = GPIO.PWM(18, 100)
pwm.start(5)


#########################################################



print('Printing X, Y, Z axis values, press Ctrl-C to quit...')


a_array = np.empty([1, 600])

initial_index = 0

#get initial alt
while initial_index <= 600:
    time.sleep(0.01)
        
    sart_time = time.time()


    # HP206C address, 0x76(118)
    # Send OSR and channel setting command, 0x44(68)

    #if index%4 == 0:
    bus.write_byte(0x76, 0x44 | 0x00)

    time.sleep(0.01)

        # HP206C address, 0x76(118)
        # Read data back from 0x10(16), 6 bytes
        # cTemp MSB, cTemp CSB, cTemp LSB, pressure MSB, pressure CSB, pressure LSB
    data = bus.read_i2c_block_data(0x76, 0x10, 6)

        # Convert the data to 20-bits
    cTemp = (((data[0] & 0x0F) * 65536) + (data[1] * 256) + data[2]) / 100.00
    fTemp = (cTemp * 1.8) + 32
    pressure = (((data[3] & 0x0F) * 65536) + (data[4] * 256) + data[5]) / 100.00

        # HP206C address, 0x76(118)
        # Send OSR and channel setting command, 0x44(68)
    bus.write_byte(0x76, 0x44 | 0x01)

    time.sleep(0.01)

        # HP206C address, 0x76(118)
        # Read data back from 0x31(49), 3 bytes
        # altitude MSB, altitude CSB, altitude LSB
    data = bus.read_i2c_block_data(0x76, 0x31, 3)

        # Convert the data to 20-bits
    altitude = (((data[0] & 0x0F) * 65536) + (data[1] * 256) + data[2]) / 100.00

    a_array[initial_index] = altitude

    initial_index+=1

alt0 = sum(a_array[20:-1])/len(a_array[20:-1])
print a_array
print "Length of alt_arr:", len(a_array)
print "alt0:", alt0


while True:

    time.sleep(0.01)
        
    sart_time = time.time()


    # HP206C address, 0x76(118)
    # Send OSR and channel setting command, 0x44(68)

    #if index%4 == 0:
    bus.write_byte(0x76, 0x44 | 0x00)

    time.sleep(0.01)

        # HP206C address, 0x76(118)
        # Read data back from 0x10(16), 6 bytes
        # cTemp MSB, cTemp CSB, cTemp LSB, pressure MSB, pressure CSB, pressure LSB
    data = bus.read_i2c_block_data(0x76, 0x10, 6)

        # Convert the data to 20-bits
    cTemp = (((data[0] & 0x0F) * 65536) + (data[1] * 256) + data[2]) / 100.00
    fTemp = (cTemp * 1.8) + 32
    pressure = (((data[3] & 0x0F) * 65536) + (data[4] * 256) + data[5]) / 100.00

        # HP206C address, 0x76(118)
        # Send OSR and channel setting command, 0x44(68)
    bus.write_byte(0x76, 0x44 | 0x01)

    time.sleep(0.01)

        # HP206C address, 0x76(118)
        # Read data back from 0x31(49), 3 bytes
        # altitude MSB, altitude CSB, altitude LSB
    data = bus.read_i2c_block_data(0x76, 0x31, 3)

        # Convert the data to 20-bits
    altitude = (((data[0] & 0x0F) * 65536) + (data[1] * 256) + data[2]) / 100.00

    x, y, z = accel.read()


    #print "%.2f," %(sart_time)
    #print('{0}, {1}, {2},'.format(x, y, z))
        
        # Output data to screen
    #print "%.2f," %altitude #altitude m
    #print "%.2f," %pressure #pressure Pa
    #print "%.2f," %fTemp #fTemp

    

###############################################################################

    #servo control
    
    if abs(altitude-alt0) < 200.0 and index >= 2:
        #move servo
        angle = 0
        duty = float(angle) / 10.0 + 2.5
        pwm.ChangeDutyCycle(duty)
    else:
        #servo position
        angle = 90
        duty = float(angle) / 10.0 + 2.5
        pwm.ChangeDutyCycle(duty)

    

###############################################################################

    print sart_time, float(x), float(y), float(z), altitude, pressure, fTemp, angle  

    index+=1

    #else:
        # Read the X, Y, Z axis acceleration values and print them.
        #x, y, z = accel.read()
        #print('{0}, {1}, {2}'.format(x, y, z))
        # Wait half a second and repeat.
        # Output data to screen
        #print "Altitude : Nan m" #%altitude
        #print "Pressure : Nan Pa" #%pressure
        #print "Temperature in Celsius : Nan C" #%cTemp
        #print "Temperature in Fahrenheit : Nan F" #%fTemp
        #print "Iteration : %.2f num" %index

        #index+=1

        


#########################################################
    #time.sleep(0.1)


