import numpy as np

data = []
with open("imu.csv") as csv:
    csv.readline()
    while len(line := csv.readline()) > 0:
            data.append([ float(i) for i in line.split(",")[0:11]])


data = np.array(data)



mx =data[:,8]
my =data[:,9]
mz =data[:,10]

mx_min = np.sort(mx)[5]
mx_max = np.sort(mx)[-5]

my_min = np.sort(my)[5]
my_max = np.sort(my)[-5]

mz_min = np.sort(mz)[5]
mz_max = np.sort(mz)[-5]


offset_x = (mx_min + mx_max) / 2
print("mx offset = {:e}".format(offset_x))


offset_y = (my_max + my_min) / 2

print("my offset = {:e}".format(offset_y))

offset_z = (mz_max + mz_min) / 2
print("mz offset = {:e}".format(offset_z))

scale_x = (mx_max-mx_min) / 2
scale_y = (my_max-my_min) / 2
scale_z = (mz_max-mz_min) / 2

scale_avg = (scale_x + scale_y + scale_z) / 3.0

scale_x = scale_avg / scale_x
scale_y = scale_avg / scale_y
scale_z = scale_avg / scale_z

print("mx scale = {:e}".format(scale_x))
print("my scale = {:e}".format(scale_y))
print("mz scale = {:e}".format(scale_z))



mx = (mx - offset_x) * scale_x
my = (my - offset_y) * scale_y
mz = (mz - offset_z) * scale_z



with open("calibrated.csv", "w") as calibrated:
    for i in range(mx.shape[0]):
        calibrated.write("{:e}, {:e}, {:e},\n".format(mx[i],my[i], mz[i]))
