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

offset_x = (max(mx) + min(mx)) / 2
print("mx offset = {:e}".format(offset_x))


offset_y = (max(my) + min(my)) / 2
print("my offset = {:e}".format(offset_y))

offset_z = (max(mz) + min(mz)) / 2
print("mz offset = {:e}".format(offset_z))