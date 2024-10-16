import serial
#from time import sleep
#import glob
import serial.tools.list_ports

SEND_LENTH = 64
def PC_host():
    #my_serial = find_ports()
    my_serial = auto_find_ports()
    if my_serial is None:
        print("No Device")
        return
    while True:
        try:
            flag=int(input('0 exit 1 update firmware 2 reset '))
        except:                     # 其他錯誤，執行這邊的程式
            print('Please enter number.')
            break
        if flag==0:
            break
        elif flag==1:
            firmware_update(my_serial)
        elif flag==2:
            send_command(my_serial, 0x10)
            #print("reset")
        else:
            print('Please enter right number.')
                   
        

def auto_find_ports():
    ports = serial.tools.list_ports.comports()
    for p in ports:
#        print(p)find port fail enter error
#        print(p.pid)
#        print(p.device)
        if p.pid == 14158:
            PATH = p.device
            my_serial = serial.Serial(port=PATH, baudrate=9600, timeout=3)
            return my_serial
    
                  

def send_command(my_serial, id):
    quotient = id//0xff #無條件捨去除法 /會有餘數
    remainder = id%0xff
    send_data = []
    can_id=bytes([quotient]) #'hello'.encode('utf-8')
    send_data.extend(can_id)
    can_id=bytes([remainder])
    send_data.extend(can_id)
    length = 66 - len(send_data)
    for j in range(length):        
        send_data.extend([0])
    receive_msg = send_recv_msg(my_serial, send_data)
    if receive_msg != '' :
        print("receive : ",receive_msg)
        return
    else:
        print("The connection cut out")
        return
    
def firmware_update(my_serial):
    #file_path=glob.glob('./SIEG_POWER.bin')
    #with open('./SIEG_POWER.bin','rb') as f:
    #with open('/home/zachary/sieg_test/SIEG_POWER.bin','rb') as f:
    #with open('/home/zachary/STM32CubeIDE/workspace_1.15.1/SIEG_POWER/STM32CubeIDE/Debug/SIEG_POWER.bin','rb') as f:
    with open('/home/zachary/STM32CubeIDE/workspace_1.15.0/qet6-test-app/Debug/qet6-test-app.bin','rb') as f:
    #with open('/home/zachary/STM32CubeIDE/workspace_1.15.0/qet6-test-app/Debug/app-test-crc.bin','rb') as f:
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
            receive_msg = send_recv_msg(my_serial, send_data) 
            if receive_msg != '' :
                print("receive : ",receive_msg)
            else:
                print("The connection cut out")
                break
#            if receive_msg[0] != 0x41 or receive_msg[1] != 0x43 or receive_msg[2] != 0x4b :
#                break#here is a bug

def send_recv_msg(my_serial, send_data):       
        my_serial.write(send_data)
        return my_serial.read(size=8)#超過3秒timeout error

if __name__ == '__main__':
    PC_host()