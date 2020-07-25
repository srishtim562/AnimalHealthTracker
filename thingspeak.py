#Thingspeak code to upload dummy data to Thingspeak cloud. 
#It can be modified to upload actual sensor data

import sys
from time import sleep
from urllib.request import urlopen

a = 37
b = 0.1

baseURL = 'http://api.thingspeak.com/update?api_key=O57IMD85QQM8J4S8&field1='

while(a < 1000):
	print(a)
	f = urlopen(baseURL +str(a))
	f.read()
	f.close()
	sleep(1)
	a = a + b	
print("Program has ended")
