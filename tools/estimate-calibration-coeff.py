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

print(max(mx), min(mx))
offset_x = (max(mx) + min(mx)) / 2
print("mx offset = {:e}".format(offset_x))


print(max(my), min(my))
offset_y = (max(my) + min(my)) / 2
print("my offset = {:e}".format(offset_y))

print(max(mz), min(mz))
offset_z = (max(mz) + min(mz)) / 2
print("mz offset = {:e}".format(offset_z))