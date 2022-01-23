# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_icm20x

i2c = board.I2C()  # uses board.SCL and board.SDA
icm = adafruit_icm20x.ICM20948(i2c)


time.sleep(1)
print("Starting..")
start = time.time()
for i in range(200):
    icm._bank = 0
    a = icm.acceleration
    #time.sleep(0.05)
    #print(a)
    #g = icm.gyro
    #m = icm.magnetic
    

print(1/ ((time.time()-start)/200))

time.sleep(1)
print("Starting..")
start = time.time()
for i in range(200):
    #a = icm.acceleration
    g = icm.gyro
    #m = icm.magnetic
    

print(1/ ((time.time()-start)/200))


time.sleep(1)
print("Starting..")
start = time.time()
for i in range(200):
    #a = icm.acceleration
    #g = icm.gyro
    m = icm.magnetic
    

print(1/ ((time.time()-start)/200))
