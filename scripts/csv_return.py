#!/usr/bin/env python2
import csv
import math

lat_setpoint = list()
long_setpoint = list()
point = list()

# lat and long acoording to ['X1 ', 'Y1 ', 'Z3 ', 'Z2', 'Z1 ', 'X2 ', 'Y3 ', 'Y2 ', 'X3']
build_lat = ['18.999950313','18.999950313' ,'18.999963864' ,'18.999963864','18.9999367615','18.999950313','18.999963864','18.9999367615','18.9999367615' ]
build_long = ['72.000170953','72.000142461','72.000170953','72.000156707','72.000142461','72.000156707','72.000142461','72.000156707','72.000170953']
i = 0
dist = 0
with open('original.csv', 'r') as file:
    reader = csv.reader(file)
    for row in reader:
        # print(row)
        if row[0] == "RETURN" :
            point.append(row[2])
            row.pop(0)
            print(row)
            row =row[0].split(";")
            # print(row)        
            lat_setpoint.append(row[0])
            long_setpoint.append(row[1])
    while i<9:
        dist = math.sqrt(math.pow(110692.0702932625 * (float(lat_setpoint[i]) - float(build_lat[i])) , 2) + math.pow(105292.0089353767 * (float(long_setpoint[i]) - float(build_long[i])) , 2))
        print(dist)
        i +=1
    # print(lat_setpoint,long_setpoint,point)