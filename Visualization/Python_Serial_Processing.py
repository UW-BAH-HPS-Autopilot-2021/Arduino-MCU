import serial
import time
Arduino_COM = 'COM3'
Arduino_Baudrate = 38400
Arduino_Serial = serial.Serial( Arduino_COM, Arduino_Baudrate )
time.sleep(5)
good_to_read = False

while ( True ):
    while( Arduino_Serial.inWaiting() == 0 ):
        pass
    data = Arduino_Serial.readline()
    data_str = str(data, 'utf-8')
    if( False == good_to_read ):
        print("Data: ", data_str, "Length", len(data_str))
    if( data_str == "Begin Outputting Data!\r\n"):
        good_to_read = True
    if( good_to_read ):
        split_data = data_str.split(',')
        if( 6 == len(split_data) ):
        # Assumption is data is in the following format:
        # "ax, ay, az, gx, gy, gz, mx, my, mz \n\r"
            ax = float(split_data[0])
            ay = float(split_data[1])
            az = float(split_data[2])
            gx = float(split_data[3])
            gy = float(split_data[4])
            gz = float(split_data[5])
            mx = float(split_data[6])
            my = float(split_data[7])
            mz = float(split_data[8])
            print("Ax: ", ax, ", Ay: ", ay, ", Az: ", az, ", Gx: ", gx, ", Gy: ", gy, ", Gz: ", gz, ", Mx: ", mx, ", My: ", my, ", Mz: ", mz)