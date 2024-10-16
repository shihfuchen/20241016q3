import pip
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    pip.main(['install', 'pyusb']) #或os.system("pip install" + "pyusb")
    import serial
    import serial.tools.list_ports
#from time import sleep
#import glob
#import serial.tools.list_ports

SEND_LENTH = 64
def PC_host():
    #my_serial = find_ports()
    my_serial = auto_find_ports()
    if my_serial is None:
        print("No Device")
        return
    while True:
        try:
            flag=int(input('0 exit 1 recv 2 send 3 send-and-recv 4 firmware-update '))    
        except:                     # 其他錯誤，執行這邊的程式
            print('Please enter Command')
            break               
        if flag==0:
            break
        elif flag==1:
            receive_msg = my_serial.read(size=8)
            if receive_msg == b'' :
                print("The connection cut out")
            else:
                print("receive : ",receive_msg)               
                    
        elif flag==2:
            try:
                CanID=int(input('Input CanID '), 16) 
            except:                     # 其他錯誤，執行這邊的程式
                print('Please enter ID')
                break 
            try:
                Data=input('Input data ').split()#encode
            except:
                print('No input data.')
            Data_dec=[]
            for x in Data:
                Data_dec.extend( x.encode('utf-8') )#不能用append會error因為append不會打開
            read = 0
            send_command(my_serial, CanID, Data_dec, read)
            #send_command(my_serial, CanID, send_data.encode('utf-8'))
            #receive_msg = send_command(my_serial, 0x07)

        elif flag==3:
            try:
                CanID=int(input('Input CanID '), 16) 
            except:                     # 其他錯誤，執行這邊的程式
                print('Please enter ID')
                break  
            #Data=input('Input data ')
            try:
                Data=input('Input data ').split()#encode
            except:
                print('No input data.')
            #print(Data)#['1', '2', '3']
            Data_dec=[]
            for x in Data:
                #print(type(x.encode('utf-8')))#<class 'bytes'>
                Data_dec.extend( x.encode('utf-8') )#不能用append會error因為append不會打開
            #Data_dec = [x.encode('utf-8') for x in Data] 
            #print(type(Data))    #bytes
            #print(type(Data_dec))    #會變list
            read = 1
            send_command(my_serial, CanID, Data_dec, read)
            #send_command(my_serial, CanID, Data.encode('utf-8'), read)

        elif flag==4:
            firmware_update(my_serial)   
        else:
            print('Error Command')    

#def find_ports():
#    ports = glob.glob('/dev/ttyACM[0-9]')
#    for port in ports:
#        my_serial = serial.Serial(port,9600)
 #       if my_serial.isOpen():
 #           break
#    my_serial = serial.Serial(port='/dev/ttyACM1',baudrate=9600, timeout=3)
#    return my_serial

def auto_find_ports():
    ports = serial.tools.list_ports.comports()
    for p in ports:
#        print(p)find port fail enter error
#        print(p.pid)
#        print(p.device)
        if p.pid == 14158:
            PATH = p.device
            #print(PATH)#COM5
            my_serial = serial.Serial(port=PATH, baudrate=9600, timeout=3)
            return my_serial    
                  
def send_command(my_serial, id, input_data, read):
    #print(type(input_data))#<class 'list'>
    #print(input_data)#[49, 50, 51]
    quotient = id//0xff #無條件捨去除法 /會有餘數
    remainder = id%0xff
    send_data = []
    can_id=bytes([quotient]) #'hello'.encode('utf-8')
    send_data.extend(can_id)
    can_id=bytes([remainder])
    send_data.extend(can_id)
    if input_data is not None:
        send_data.extend(input_data)
        #print(send_data)#[0, 128, 49, 50, 51]
    length = 66 - len(send_data)
    for j in range(length):        
        send_data.extend([0])
    #print(type(send_data))#<class 'list'>
    receive_msg = send_recv_msg(my_serial, send_data, read)
    if receive_msg == b'' :        
        print("The connection cut out")
        return
    else:
        print("receive : ",receive_msg)
        return
    
def firmware_update(my_serial):
    #file_path=glob.glob('./SIEG_POWER.bin')
    #with open('./app-test-crc.bin','rb') as f:
    #with open('./SIEG_POWER.bin','rb') as f:
    #with open('/home/zachary/sieg-test-again/SIEG_POWER/STM32CubeIDE/Debug/SIEG_POWER.bin','rb') as f:
    #with open('/home/zachary/STM32CubeIDE/workspace_1.15.1/SIEG_POWER/STM32CubeIDE/Debug/SIEG_POWER.bin','rb') as f:
    #with open('/home/zachary/STM32CubeIDE/workspace_1.15.0/qet6-test-app/Debug/qet6-test-app.bin','rb') as f:
    #with open('/home/zachary/STM32CubeIDE/workspace_1.15.1/SIEG_PWRBL/Debug/test6.bin','rb') as f:
    with open('/home/zachary/STM32CubeIDE/workspace_1.15.1/SIEG_PWRBL/Debug/test7iwdg.bin','rb') as f:
        read_data = f.read() #read all #<class 'bytes'>
        size = len(read_data)
        #data_byte = data.to_bytes(4, byteorder='little', signed=True)
        for i in range(0, size, SEND_LENTH):
            #print(i)
            send_data = []
            if (i+SEND_LENTH) < size:
                can_id=bytes([0x0])
                send_data.extend(can_id)
                can_id=bytes([0x5])
                send_data.extend(can_id)
                send_data.extend(read_data[i:i+SEND_LENTH])
            else:
                can_id=bytes([0x0])
                send_data.extend(can_id)
                can_id=bytes([0x6])
                offset=i+SEND_LENTH-size
                send_data.extend(can_id)
                send_data.extend(read_data[i:i+SEND_LENTH])
                for j in range(offset):
                    send_data.extend([0])
            #print(send_data)
            receive_msg = send_recv_msg(my_serial, send_data, 1) 
            if receive_msg == b'' :                
                print("The connection cut out")
                break
            else:
                print("receive : ",receive_msg)
#            if receive_msg[0] != 0x41 or receive_msg[1] != 0x43 or receive_msg[2] != 0x4b :
#                break#here is a bug

def send_recv_msg(my_serial, send_data, read):
#    my_serial = serial.Serial('/dev/ttyACM1', 9600)#linux '/dev/ttyUSB0' or '/dev/ttyACM0' #windows 'COM3', 9600
#    if my_serial.isOpen():
#        nak_times = 0
        
        my_serial.write(send_data)
        if read==1:
            return my_serial.read(size=8)#超過3秒timeout error
#        receive_msg = my_serial.read(size=8)#超過3秒timeout error
#        return receive_msg
#        if receive_msg != '' :
#            print("receive : ",receive_msg)
#            return receive_msg
#        else:
#            print("The connection cut out")
#            return 
        #print("send_data")
#        while True:
#            receive_msg = my_serial.read(size=8)
#            if receive_msg != '' :
#                print("receive : ",receive_msg)
#                return receive_msg
#            else:
#                nak_times+=1
#                print("receive none")
#                sleep(0.2)
#                if nak_times>=3:
#                    print("The connection cut out")
#                    return [0,0,0]
#    else :
#        print("open failed")
#        return [0,0,0]

if __name__ == '__main__':
    PC_host()