#!/usr/bin/env python2
import csv
import math

lat_setpoint = list()
long_setpoint = list()
alt_setpoint = list()

# lat and long acoording to ['C2', 'B2', 'B1', 'C3', 'C1', 'A1', 'A3', 'B3', 'A2']
build_lat = ['18.999837387', '18.999823836' ,'18.999837387','18.999837387','18.9998102845','18.9998102845','18.999823836','18.9998102845' ]
build_long = ['72.000156707','72.000142461','72.000170953','72.000142461','72.000142461','72.000170953','72.000170953','72.000156707']
i = 0
dist = 0
with open('manifest.csv', 'r') as file:
    reader = csv.reader(file)
    for row in reader:
        if row[0] == "DELIVERY" :
            row.pop(0)
            # print(row)
            if row[0]!="B2":
                row =row[1].split(";")
                print(row)        
                lat_setpoint.append(row[0])
                long_setpoint.append(row[1])
                alt_setpoint.append(row[2])
            else:
                continue
    while i<8:
        dist = math.sqrt(math.pow(110692.0702932625 * (float(lat_setpoint[i]) - float(build_lat[i])) , 2) + math.pow(105292.0089353767 * (float(long_setpoint[i]) - float(build_long[i])) , 2))
        # print(dist)
        i +=1
    print(lat_setpoint,long_setpoint,alt_setpoint)