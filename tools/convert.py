bad_count = 0
with open("imu_ontop_of_wood_ontop_of_robot.csv") as csv:
    with open("out.txt", "w") as out:
        for line in csv:
            values = line.split(",")

            try:
                out_line = "%f\t%f\t%f\n" %(float(values[8])*1e6, float(values[9])*1e6, float(values[10])*1e6)
                if float(values[10])*1e9 > -35000 or True:
                    print(out_line)
                    out.write(out_line)
                else:
                    bad_count+= 1
            except:
                pass
print(bad_count)
