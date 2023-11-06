import serial
import time as t

ser = serial.Serial('/dev/ttyACM0', 115200, timeout = .005)

print('Connected to:', ser.portstr)

while True:
    ser.flush()
    t.sleep(.1)
    line = ser.readline().rstrip().decode() # read the string
    str_line = line.split(',')
    if str_line != ['']:
        if str_line[0] == 'PICOBATMON':
            battVoltsStr = str_line[3].split(':')
            criticalState = str_line[4].split(':')
            
            print(str_line)
            
            battVolts = float(battVoltsStr[1])            
            critState = int(criticalState[1])
            
            print('Battery Voltage = ' + str(battVolts) + ' ' + 'CriticalState = ' + str(critState))
            
            if battVolts <= 17.0:
                print("Warning - Battery is getting low!")
            if critState == 2:
                print("Run shutdown script ASAP")
                t.sleep(2)
    
    xnums = len(str_line)
    
    