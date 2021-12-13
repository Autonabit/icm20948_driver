#!/usr/bin/env python3

import time
import board
import adafruit_icm20x

i2c = board.I2C()
icm = adafruit_icm20x.ICM20948(i2c)

t = icm.temperature
a = icm.acceleration
g = icm.gyro
m = icm.magnetic
time.sleep(0.5)

start = time.time()
with open("imu.csv", "w") as csv:
    csv.write("time, temp, ax, ay, az, gx, gy, gz, mx, my, mz,\n")
    try:
        while True:
            t = icm.temperature
            a = icm.acceleration
            g = icm.gyro
            m = tuple(i*1e-6 for i in icm.magnetic) #convert from uT to T

            #data = "%.2f, %.2f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f," %(time.time()-start, t, a[0], a[1], a[2], g[0], g[1], g[2], m[0], m[1], m[2])
            data = "{:e}, {:e}, {:e}, {:e}, {:e},{:e}, {:e}, {:e},{:e}, {:e}, {:e},".format(time.time()-start, t, a[0], a[1], a[2], g[0], g[1], g[2], m[0], m[1], m[2])
            print(data)
            csv.write(data+"\n")
            #time.sleep(0.1)
    except KeyboardInterrupt:
        print("exiting")
