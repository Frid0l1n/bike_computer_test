import smbus2
import time


# initialize icm
port_icm = 1
address_icm = 0x68 

REG_BANK_SEL = 0x7F
REG_PWR_MGMT_1 = 0x06
REG_ACCEL_XOUT_H = 0x2D
REG_ACCEL_CONFIG = 0x14

bus_icm = smbus2.SMBus(port_icm)

def switch_bank(bank):
    bus_icm.write_byte_data(address_icm, REG_BANK_SEL, bank << 4)

def initialize_icm():
    switch_bank(0)
    bus_icm.write_byte_data(address_icm, REG_PWR_MGMT_1, 0x01)
    time.sleep(0.1)

    switch_bank(2)

    bus_icm.write_byte_data(address_icm, REG_ACCEL_CONFIG, 0x00)
    time.sleep(0.1)

    switch_bank(0)

def read_accel():
    data = bus_icm.read_i2c_block_data(address_icm, REG_ACCEL_XOUT_H, 6)

    x = (data[0] << 8) | data[1]
    y = (data[2] << 8) | data[3]
    z = (data[4] << 8) | data[5]
    
    x = x - 65536 if x > 32767 else x
    y = y - 65536 if y > 32767 else y
    z = z - 65536 if z > 32767 else z

    ax = x / 16384.0
    ay = y / 16384.0
    az = z / 16384.0
    
    return ax, ay, az

initialize_icm()



while True:
    ax, ay, az = read_accel()
    print(f"Accel: X={ax:.2f}g, Y={ay:.2f}g, Z={az:.2f}g")
    time.sleep(0.5)
