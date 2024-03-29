# Reading PZEM-004t power sensor (new version v3.0) through Modbus-RTU protocol over TTL UART
# Run as:
# python3 pzem_004t.py

# To install dependencies: 
# pip install modbus-tk
# pip install pyserial

import serial
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

# Connect to the slave
serial = serial.Serial(
                       port='/dev/usbSensorCotxe',
                       baudrate=9600,
                       bytesize=8,
                       parity='N',
                       stopbits=1,
                       xonxoff=0
                      )

master = modbus_rtu.RtuMaster(serial)
master.set_timeout(2.0)
master.set_verbose(True)

data = master.execute(1, cst.READ_INPUT_REGISTERS, 0, 10)

voltage = data[0] / 10.0 # [V]
current = (data[1] + (data[2] << 16)) / 1000.0 # [A]
power = (data[3] + (data[4] << 16)) / 10.0 # [W] active power (V * I * power factor)
energy = data[5] + (data[6] << 16) # [Wh]
frequency = data[7] / 10.0 # [Hz]
powerFactor = data[8] / 100.0
alarm = data[9] # 0 = no alarm

json_body = '"volts": {}, "current": {}, "power": {}, "energy": {}'
#print('Frequency [Hz]: ', frequency)
#print('Power factor []: ', powerFactor)
#print('Alarm : ', alarm)
print('{'+json_body.format(voltage,current,power,energy)+'}')

# Changing power alarm value to 100 W
# master.execute(1, cst.WRITE_SINGLE_REGISTER, 1, output_value=100)

try:
    master.close()
    if slave.is_open:
        slave.close()
except:
    pass