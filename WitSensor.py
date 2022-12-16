#coding:UTF-8
#Before running, you need to install pyserial. 
# Use WIN+R to call out the running box, enter CMD, enter the command line, 
# and enter pip install pyserial to update the function library

import serial
import time
 
ACCData     =   [0.0]*8     #0x51   FrameState=1
GYROData    =   [0.0]*8     #0x52   FrameState=2
AngleData   =   [0.0]*8     #0x53   FrameState=3
MagData     =   [0.0]*8     #0x54   FrameState=4

BaroData    =   [0.0]*8     #0x56   FrameState=5
LonLatData =   [0.0]*8     #0x57   FrameState=6
GPSData     =   [0.0]*8     #0x58   FrameState=7
QuatData   =    [0.0]*8     #0x59   FrameState=8

FrameState  =   0           #Default state
Bytenum = 0               
CheckSum = 0                      
 
a = [0.0]*4
w = [0.0]*3
Angle = [0.0]*3
Mag = [0.0]*3
Baro = [0.0]*2
LonLat = [0.0]*4
GPS = [0.0]*3
Quat = [0.0]*4


def Get_Guad_Data(inputdata):   
    global  FrameState   
    global  Bytenum
    global  CheckSum
    global  a       # 3 double tuple
    global  w       # 3 double tuple
    global  Angle   # 3 double tuple
    global  Mag     # 3 double tuple

    global  Baro
    global  LonLat
    global  GPS
    global  Quat

    for data in inputdata:  
        #Python2软件版本这里需要插入 data = ord(data)*****************************************************************************************************
        if FrameState==0:   
            if data==0x55 and Bytenum==0: 
                CheckSum=data
                Bytenum=1
                continue
            elif data==0x51 and Bytenum==1:
                CheckSum+=data
                FrameState=1
                Bytenum=2
            elif data==0x52 and Bytenum==1:
                CheckSum+=data
                FrameState=2
                Bytenum=2
            elif data==0x53 and Bytenum==1:
                CheckSum+=data
                FrameState=3
                Bytenum=2
            elif data==0x54 and Bytenum==1:
                CheckSum+=data
                FrameState=4
                Bytenum=2
            elif data==0x56 and Bytenum==1:
                CheckSum+=data
                FrameState=5
                Bytenum=2
            elif data==0x57 and Bytenum==1:
                CheckSum+=data
                FrameState=6
                Bytenum=2
            elif data==0x58 and Bytenum==1:
                CheckSum+=data
                FrameState=7
                Bytenum=2
            elif data==0x59 and Bytenum==1:
                CheckSum+=data
                FrameState=8
                Bytenum=2

        elif FrameState==1: # acc    
            
            if Bytenum<10:            
                ACCData[Bytenum-2]=data 
                CheckSum+=data
                Bytenum+=1
            else:
                if data == (CheckSum&0xff):  
                    a = get_acc(ACCData)
                CheckSum=0                  
                Bytenum=0
                FrameState=0


        elif FrameState==2: # gyro
            
            if Bytenum<10:
                GYROData[Bytenum-2]=data
                CheckSum+=data
                Bytenum+=1
            else:
                if data == (CheckSum&0xff):
                    w = get_gyro(GYROData)
                CheckSum=0
                Bytenum=0
                FrameState=0


        elif FrameState==3: # angle
            
            if Bytenum<10:
                AngleData[Bytenum-2]=data
                CheckSum+=data
                Bytenum+=1
            else:
                if data == (CheckSum&0xff):
                    Angle = get_angle(AngleData)
                CheckSum=0
                Bytenum=0
                FrameState=0


        elif FrameState==4: # Mag
            
            if Bytenum<10:
                MagData[Bytenum-2]=data
                CheckSum+=data
                Bytenum+=1
            else:
                if data == (CheckSum&0xff):
                    Mag = get_mag(MagData)
                CheckSum=0
                Bytenum=0
                FrameState=0


        elif FrameState==5: # Bar           
            if Bytenum<10:
                BaroData[Bytenum-2]=data
                CheckSum+=data
                Bytenum+=1
            else:
                if data == (CheckSum&0xff):
                    # print('Baro Triggered',BaroData)
                    Baro = get_Bar(BaroData)
                CheckSum=0
                Bytenum=0
                FrameState=0


        elif FrameState==6: # LonLat
            
            if Bytenum<10:
                LonLatData[Bytenum-2]=data
                CheckSum+=data
                Bytenum+=1
            else:
                if data == (CheckSum&0xff):
                    LonLat = get_LonLat(LonLatData)
                CheckSum=0
                Bytenum=0
                FrameState=0


        elif FrameState==7: # GPS
            
            if Bytenum<10:
                GPSData[Bytenum-2]=data
                CheckSum+=data
                Bytenum+=1
            else:
                if data == (CheckSum&0xff):
                    GPS = get_GPS(GPSData)
                CheckSum=0
                Bytenum=0
                FrameState=0


        elif FrameState==8: # Quat
            
            if Bytenum<10:
                QuatData[Bytenum-2]=data
                CheckSum+=data
                Bytenum+=1
            else:
                if data == (CheckSum&0xff):
                    Quat = get_Quat(QuatData)
                CheckSum=0
                Bytenum=0
                FrameState=0
    # print("a(g):%10.3f %10.3f %10.3f w(deg/s):%10.3f %10.3f %10.3f Angle(deg):%10.3f %10.3f %10.3f"%d)
    print('Acc(m/s^2),Temp:',a)
    print('Angular Vel(deg/s):',w)
    print('Angle(deg):',Angle)
    print('Mag(This is relative), Temp:',Mag)
    print('Baro(This is relative),Height(m):',Baro)
    print('Lon:',LonLat[0],'degree',LonLat[1],'minute',' Lat:',LonLat[2],'degree',LonLat[3],'minute')
    print('Quat:',Quat)
 
 
def get_acc(datahex):  
    axl = datahex[0]                                        
    axh = datahex[1]
    ayl = datahex[2]                                        
    ayh = datahex[3]
    azl = datahex[4]                                        
    azh = datahex[5]
    tl  = datahex[6]
    th  = datahex[7]
    
    k_acc = 16.0 * 9.8
    k_temp = 100

    acc_x = (axh << 8 | axl) / 32768.0 * k_acc
    acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
    acc_z = (azh << 8 | azl) / 32768.0 * k_acc

    temperature = (th << 8 | tl) / 32768.0 * k_temp

    if acc_x >= k_acc:
        acc_x -= 2 * k_acc
    if acc_y >= k_acc:
        acc_y -= 2 * k_acc
    if acc_z >= k_acc:
        acc_z-= 2 * k_acc
    if temperature >= k_temp:
        temperature-= k_temp
    
    return acc_x,acc_y,acc_z,temperature
 
 
def get_gyro(datahex):                                      
    wxl = datahex[0]                                        
    wxh = datahex[1]
    wyl = datahex[2]                                        
    wyh = datahex[3]
    wzl = datahex[4]                                        
    wzh = datahex[5]
    k_gyro = 2000.0
 
    gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
    gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
    gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
    if gyro_x >= k_gyro:
        gyro_x -= 2 * k_gyro
    if gyro_y >= k_gyro:
        gyro_y -= 2 * k_gyro
    if gyro_z >=k_gyro:
        gyro_z-= 2 * k_gyro
    return gyro_x,gyro_y,gyro_z
 
 
def get_angle(datahex):                                 
    rxl = datahex[0]                                        
    rxh = datahex[1]
    ryl = datahex[2]                                        
    ryh = datahex[3]
    rzl = datahex[4]                                        
    rzh = datahex[5]
    k_angle = 180.0
 
    angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
    angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
    angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
    if angle_x >= k_angle:
        angle_x -= 2 * k_angle
    if angle_y >= k_angle:
        angle_y -= 2 * k_angle
    if angle_z >=k_angle:
        angle_z-= 2 * k_angle
 
    return angle_x,angle_y,angle_z
 
def get_mag(datahex):
    hxl = datahex[0]
    hxh = datahex[1]
    hyl = datahex[2]
    hyh = datahex[3]
    hzl = datahex[4]
    hzh = datahex[5]

    # txl = datahex[6]
    # txh = datahex[7]
    
    k_mag   = 30
    # k_tempe  = 100.0
    

    mag_x =         (hxh << 8 | hxl )/ 32768.0 * k_mag
    mag_y =         (hyh << 8 | hyl) / 32768.0 * k_mag
    mag_z =         (hzh << 8 | hzl )/ 32768.0 * k_mag
    # temperature =   (txh << 8 | txl) / 32768.0 * k_tempe

    # print('Temp:',  txl)

    if mag_x >= k_mag:
        mag_x -= 2 * k_mag
    if mag_y >= k_mag:
        mag_y -= 2 * k_mag
    if mag_z >=k_mag:
        mag_z -= 2 * k_mag
    # if temperature >= k_tempe:
    #     temperature-= 2 * k_tempe

    return mag_x,mag_y,mag_z

def get_Bar(datahex):
    P0 = datahex[0]
    P1 = datahex[1]
    P2 = datahex[2]
    P3 = datahex[3]

    H0 = datahex[4]
    H1 = datahex[5]
    H2 = datahex[6]
    H3 = datahex[7]

    B_P = 2147483648.0 
    B_H = 2147483648.0 

    P = ( (P3 << 24) |(P2 << 16) |(P1 << 8) | P0) * 1.0
    H = ( (H3 << 24) |(H2 << 16) |(H1 << 8) | H0) * 1.0
    # print('P',P,'H',H)

    if P >= B_P:
        P -= 2 * B_P
    if H >= B_H:
        H -= 2 * B_H

    # print('P',P,'H',H)
    return P, H


def get_LonLat(datahex):
    Lon0 = datahex[0]
    Lon1 = datahex[1]
    Lon2 = datahex[2]
    Lon3 = datahex[3]

    Lat0 = datahex[4]
    Lat1 = datahex[5]
    Lat2 = datahex[6]
    Lat3 = datahex[7]

    B_L = 2147483648.0 

    Lon = ((Lon3 << 24) |(Lon2 << 16) |(Lon1 << 8) | Lon0) * 1.0
    Lat = ((Lat3 << 24) |(Lat2 << 16) |(Lat1 << 8) | Lat0) * 1.0

    if Lon >= B_L:
        Lon -= 2 * B_L
    if Lat >= B_L:
        Lat -= 2 * B_L

    Lon_d = Lon / 10000000
    Lat_d = Lat / 10000000

    Lon_m = (Lon % 10000000) / 100000
    Lat_m = (Lat % 10000000) / 100000

    return Lon_d, Lon_m, Lat_d, Lat_m 

def get_GPS(datahex):
    Heightl = datahex[0]
    Heighth = datahex[1]

    Yawl    = datahex[2]
    Yawh    = datahex[3]

    GPSV0   = datahex[4]
    GPSV1   = datahex[5]
    GPSV2   = datahex[6]
    GPSV3   = datahex[7]

    k_H     = 0.1
    k_Yaw   = 0.01
    k_GPSV  = 0.001

    Height  = (Heighth << 8 | Heightl) * k_H
    Yaw     = (Yawh << 8 | Yawl) * k_Yaw
    GPSV    = ((GPSV3 << 24) |(GPSV2 << 16) |(GPSV1 << 8) | GPSV0) * k_GPSV 

    if Height >= 32768.0 * k_H:
        Height -= 2 * 32768.0 * k_H
    if Yaw >= 32768.0 * k_Yaw:
        Yaw -= 2 * 32768.0 * k_Yaw
    if GPSV >= 2147483648.0  * k_GPSV:
        GPSV -= 2 * 2147483648.0  * k_GPSV

    return   Height, Yaw, GPSV


def get_Quat(datahex):
    Q0l = datahex[0]
    Q0h = datahex[1]

    Q1l = datahex[2]
    Q1h = datahex[3]

    Q2l = datahex[4]
    Q2h = datahex[5]

    Q3l = datahex[6]
    Q3h = datahex[7]

    Q0  = (Q0h << 8 | Q0l) / 32768.0
    Q1  = (Q1h << 8 | Q1l) / 32768.0
    Q2  = (Q2h << 8 | Q2l) / 32768.0
    Q3  = (Q3h << 8 | Q3l) / 32768.0

    if Q0 >= 1:
        Q0 -= 2
    if Q1 >= 1:
        Q1 -= 2
    if Q2 >= 1:
        Q2 -= 2
    if Q3 >= 1:
        Q3 -= 2

    return Q0, Q1, Q2, Q3

if __name__=='__main__': 
    # use raw_input function for python 2.x or input function for python3.x
    port = input('please input port No. such as com7:');                #Python2软件版本用    port = raw_input('please input port No. such as com7:');*****************************************************************************************************
    #port = input('please input port No. such as com7:'));
    baud = int(input('please input baudrate(115200 for JY61 or 9600 for JY901):'))
    ser = serial.Serial(port, baud, timeout=0.5)  # ser = serial.Serial('com7',115200, timeout=0.5) 
    print(ser.is_open)
    while(1):
        ticks = time.time()
        datahex = ser.read(99) # Per data 11 Byte, and 51 - 5A for nine items, total 9 * 11
        Get_Guad_Data(datahex)      
        tocks = time.time()
        print('Duetime:',tocks - ticks)   
