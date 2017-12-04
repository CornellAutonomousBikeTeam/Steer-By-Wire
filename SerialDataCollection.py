import serial
import datetime
import time

ser = serial.Serial('/dev/ttyACM0',115200)
while True:
    prettyTime = datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d %H:%M:%S')
    name = 'SBW_Tests/test'+prettyTime+'.csv'
    f = open(name,"w+")
    print "new file"
    while True:
        data = ser.readline()
        if "Start" in data:
            f.close() 
            break
        print data
        f.write(data)


f.close()

