import serial
import struct

START_BYTE = 0xBB
READ_QUAT = 0x01
READ_RPY = 0x02
READ_RPY_VAR = 0x03
WRITE_RPY_VAR = 0x04
READ_ACC = 0x05
READ_ACC_RAW = 0x06
READ_ACC_OFF = 0x07
WRITE_ACC_OFF = 0x08
READ_ACC_VAR = 0x09
WRITE_ACC_VAR = 0x0A
READ_GYRO = 0x0B
READ_GYRO_RAW = 0x0C
READ_GYRO_OFF = 0x0D
WRITE_GYRO_OFF = 0x0E
READ_GYRO_VAR = 0x0F
WRITE_GYRO_VAR = 0x10
READ_MAG = 0x11
READ_MAG_RAW = 0x12
READ_MAG_H_OFF = 0x13
WRITE_MAG_H_OFF = 0x14
READ_MAG_S_OFF0 = 0x15
WRITE_MAG_S_OFF0 = 0x16
READ_MAG_S_OFF1 = 0x17
WRITE_MAG_S_OFF1 = 0x18
READ_MAG_S_OFF2 = 0x19
WRITE_MAG_S_OFF2 = 0x1A
SET_I2C_ADDR = 0x1B
GET_I2C_ADDR = 0x1C
SET_FILTER_GAIN = 0x1D
GET_FILTER_GAIN = 0x1E
SET_FRAME_ID = 0x1F
GET_FRAME_ID = 0x20
RESET_PARAMS = 0x21
READ_QUAT_RPY = 0x22
READ_ACC_GYRO = 0x23




class EIMU_V2_FULL:
    def __init__(self, port, baud=921600, timeOut=0.1):
        self.ser = serial.Serial(port, baud, timeout=timeOut)
    
    #------------------------------------------------------------------------
    def send_packet_without_payload(self, cmd):
        length = 0
        packet = bytearray([START_BYTE, cmd, length])
        checksum = sum(packet) & 0xFF
        packet.append(checksum)
        self.ser.write(packet)

    def send_packet_with_payload(self, cmd, payload_bytes):
        length = len(payload_bytes)
        packet = bytearray([START_BYTE, cmd, length]) + payload_bytes
        checksum = sum(packet) & 0xFF
        packet.append(checksum)
        self.ser.write(packet)

    def read_packet1(self):
        payload = self.ser.read(4)
        a = struct.unpack('<f', payload)[0]  # little-endian float
        return a
    
    def read_packet3(self):
        payload = self.ser.read(12)
        a, b, c = struct.unpack('<fff', payload)  # little-endian float
        return a, b, c

    def read_packet4(self):
        payload = self.ser.read(16)
        a, b, c, d = struct.unpack('<ffff', payload)  # little-endian float
        return a, b, c, d
    
    def read_packet6(self):
        payload = self.ser.read(24)
        a, b, c, d, e, f = struct.unpack('<ffffff', payload)  # little-endian float
        return a, b, c, d, e, f
    
    def read_packet8(self):
        payload = self.ser.read(32)
        a, b, c, d, e, f, g, h = struct.unpack('<ffffffff', payload)  # little-endian float
        return a, b, c, d, e, f, g, h
    
    #---------------------------------------------------------------------

    def write_data1(self, cmd, pos, val):
        payload = struct.pack('<Bf', pos, val)
        self.send_packet_with_payload(cmd, payload)
        val = self.read_packet1()
        return val

    def read_data1(self, cmd, pos):
        payload = struct.pack('<Bf', pos, 0.0)  # big-endian
        self.send_packet_with_payload(cmd, payload)
        val = self.read_packet1()
        return val
    
    def write_data3(self, cmd, a, b, c):
        payload = struct.pack('<fff', a,b,c) 
        self.send_packet_with_payload(cmd, payload)
        val = self.read_packet1()
        return val

    def read_data3(self, cmd):
        self.send_packet_without_payload(cmd)
        a, b, c = self.read_packet3()
        return a, b, c

    def write_data4(self, cmd, a, b, c, d):
        payload = struct.pack('<ffff', a,b,c,d) 
        self.send_packet_with_payload(cmd, payload)
        val = self.read_packet1()
        return val

    def read_data4(self, cmd):
        self.send_packet_without_payload(cmd)
        a, b, c, d = self.read_packet4()
        return a, b, c, d
    
    def read_data6(self, cmd):
        self.send_packet_without_payload(cmd)
        a, b, c, d, e, f = self.read_packet6()
        return a, b, c, d, e, f
    
    def read_data8(self, cmd):
        self.send_packet_without_payload(cmd)
        a, b, c, d, e, f, g, h = self.read_packet8()
        return a, b, c, d, e, f, g, h
        
    #---------------------------------------------------------------------

    def setWorldFrameId(self, id):
        res = self.write_data1(SET_FRAME_ID, 0, id)
        return int(res)
    
    def getWorldFrameId(self):
        id = self.read_data1(GET_FRAME_ID, 0)
        return int(id)
    
    def getFilterGain(self):
        gain = self.read_data1(GET_FILTER_GAIN, 0)
        return round(gain,6)
    
    def readQuat(self):
        qw, qx, qy, qz = self.read_data4(READ_QUAT)
        return round(qw,6), round(qx,6), round(qy,6), round(qz,6)
    
    def readRPY(self):
        r, p, y = self.read_data3(READ_RPY)
        return round(r,6), round(p,6), round(y,6)
    
    def readRPYVariance(self):
        r, p, y = self.read_data3(READ_RPY_VAR)
        return round(r,8), round(p,8), round(y,8)
    
    def readAcc(self):
        ax, ay, az = self.read_data3(READ_ACC)
        return round(ax,6), round(ay,6), round(az,6)
    
    def readAccVariance(self):
        ax, ay, az = self.read_data3(READ_ACC_VAR)
        return round(ax,8), round(ay,8), round(az,8)
    
    def readGyro(self):
        gx, gy, gz = self.read_data3(READ_GYRO)
        return round(gx,6), round(gy,6), round(gz,6)
    
    def readGyroVariance(self):
        gx, gy, gz = self.read_data3(READ_GYRO_VAR)
        return round(gx,8), round(gy,8), round(gz,8)
    
    def readMag(self):
        mx, my, mz = self.read_data3(READ_MAG)
        return round(mx,6), round(my,6), round(mz,6)

    ###################################################

    def setI2cAddress(self, i2cAddress):
        res = self.write_data1(SET_I2C_ADDR, 0, i2cAddress)
        return int(res)
    
    def getI2cAddress(self):
        i2cAddress = self.read_data1(GET_I2C_ADDR, 0)
        return int(i2cAddress)
    
    def resetAllParams(self):
        res = self.write_data1(RESET_PARAMS, 0, 0.0)
        return int(res)

    def setFilterGain(self, gain):
        res = self.write_data1(SET_FILTER_GAIN, 0, gain)
        return int(res)
    
    def writeRPYVariance(self, r, p, y):
        res = self.write_data3(WRITE_RPY_VAR, r, p, y)
        return int(res)
    
    def readAccRaw(self):
        ax, ay, az = self.read_data3(READ_ACC_RAW)
        return round(ax,6), round(ay,6), round(az,6)
    
    def readAccOffset(self):
        ax, ay, az = self.read_data3(READ_ACC_OFF)
        return round(ax,6), round(ay,6), round(az,6)
    
    def writeAccOffset(self, ax, ay, az):
        res = self.write_data3(WRITE_ACC_OFF, ax, ay, az)
        return int(res)
    
    def writeAccVariance(self, ax, ay, az):
        res = self.write_data3(WRITE_ACC_VAR, ax, ay, az)
        return int(res)
    
    def readGyroRaw(self):
        gx, gy, gz = self.read_data3(READ_GYRO_RAW)
        return round(gx,6), round(gy,6), round(gz,6)
    
    def readGyroOffset(self):
        gx, gy, gz = self.read_data3(READ_GYRO_OFF)
        return round(gx,6), round(gy,6), round(gz,6)
    
    def writeGyroOffset(self, gx, gy, gz):
        res = self.write_data3(WRITE_GYRO_OFF, gx, gy, gz)
        return int(res)
    
    def writeGyroVariance(self, gx, gy, gz):
        res = self.write_data3(WRITE_GYRO_VAR, gx, gy, gz)
        return int(res)
    
    def readMagRaw(self):
        mx, my, mz = self.read_data3(READ_MAG_RAW)
        return round(mx,6), round(my,6), round(mz,6)
    
    def readMagHardOffset(self):
        mx, my, mz = self.read_data3(READ_MAG_H_OFF)
        return round(mx,6), round(my,6), round(mz,6)
    
    def writeMagHardOffset(self, mx, my, mz):
        res = self.write_data3(WRITE_MAG_H_OFF, mx, my, mz)
        return int(res)
    
    def readMagSoftOffset0(self):
        mx, my, mz = self.read_data3(READ_MAG_S_OFF0)
        return round(mx,6), round(my,6), round(mz,6)
    
    def writeMagSoftOffset0(self, mx, my, mz):
        res = self.write_data3(WRITE_MAG_S_OFF0, mx, my, mz)
        return int(res)
    
    def readMagSoftOffset1(self):
        mx, my, mz = self.read_data3(READ_MAG_S_OFF1)
        return round(mx,6), round(my,6), round(mz,6)
    
    def writeMagSoftOffset1(self, mx, my, mz):
        res = self.write_data3(WRITE_MAG_S_OFF1, mx, my, mz)
        return int(res)
    
    def readMagSoftOffset2(self):
        mx, my, mz = self.read_data3(READ_MAG_S_OFF2)
        return round(mx,6), round(my,6), round(mz,6)
    
    def writeMagSoftOffset2(self, mx, my, mz):
        res = self.write_data3(WRITE_MAG_S_OFF2, mx, my, mz)
        return int(res)
    
    #---------------------------------------------------------------------

    def readQuatRPY(self):
        qw, qx, qy, qz, r, p, y, _ = self.read_data8(READ_QUAT_RPY)
        return round(qw,6), round(qx,6), round(qy,6), round(qz,6), round(r,6), round(p,6), round(y,6)
    
    def readAccGyro(self):
        ax, ay, az, gx, gy, gz = self.read_data6(READ_ACC_GYRO)
        return round(ax,6), round(ay,6), round(az,6), round(gx,6), round(gy,6), round(gz,6)