import serial
import time

# Open serial port (adjust 'COM3' to your port)
ser = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=1)

# Wait for a moment before sending data
time.sleep(2)

# Send data
data = [1.2, 3.4, 5.6, 7.8, 9.0, 1.1, 2.3, 4.5, 6.7, 8.9, 1.0]
if(len(data)<100):
    while(len(data)<100):
        data.append(0.0)

data_str = ','.join(str(x) for x in data) + '\n'

print(data_str)
ser.write(data_str.encode())

# Close serial port
ser.close()
