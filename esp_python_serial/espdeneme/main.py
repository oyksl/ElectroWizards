import serial
import time

# Open serial port (adjust 'COM3' to your port)
ser = serial.Serial("COM3", 115200, timeout=1)
serial.Serial()
# Wait for a moment before sending data
time.sleep(2)

# Send data

data = [[100,50],[100,100],[100,150],[100,200],[100,250]]


#while(1):
for i in range(1,5):
    data_str = ','.join(str(x) for x in data[i]) + '\n'
    ser.write(data_str.encode())
    time.sleep(3)
    print(data_str)


ser.write(data_str.encode())

# Close serial port
ser.close()
