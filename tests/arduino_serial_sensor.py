# From https://www.learnrobotics.org/blog/arduino-data-logger-csv/
import serial
import csv

arduino_port = "/dev/ttyACM0" #serial port of Arduino
baud = 9600 #arduino uno runs at 9600 baud
fileName = "analog-data.csv" #name of the CSV file generated
ser = serial.Serial(arduino_port, baud)

print("Connected to Arduino port:" + arduino_port)
#file = open(fileName, "a")
print("Created file")

#display the data to the terminal
getData=ser.readline()
dataString = getData.decode('utf-8')
data=dataString[0:][:-2]
print(data)

readings = data.split(",")
print(readings)
