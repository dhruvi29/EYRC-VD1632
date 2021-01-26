#!/usr/bin/env python2
import csv

lat_setpoint = list()
long_setpoint = list()
alt_setpoint = list()


with open('manifest.csv', 'r') as file:
	reader = csv.reader(file)
	for row in reader:
		row.pop(0)
		lat_setpoint.append(row[0])
		long_setpoint.append(row[1])
		alt_setpoint.append(row[2])
	print(lat_setpoint,long_setpoint,alt_setpoint)
