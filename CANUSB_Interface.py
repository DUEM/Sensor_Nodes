
import serial


#open port
ser = serial.Serial()
ser.port = "COM7"
ser.baudrate = 115200
ser.timeout = 1
ser.open()
if(ser.isOpen()):
    print("Port opened")
    #ser.write(r"S6\r") should set canusb speed of 500kbit/s
    #ser.write(r"O\R") should open canusb
    print("canbus opened")

else:
    print( "Port did not open")

#when something appears on canbus sort and store item
t=0
while t<10: #arbitrary number to ensure port eventually closes
    print("i got here")
    x= ser.read(22)
    print(x)
    y=list(x)
    if x != "":
        joined = "0x"+y[1]+y[2]+y[3]
        node_id = int(joined,0)
        data=[]
        for j in range(5,21,2):
            data.append(int("0x"+y[j]+y[j+1],0))
        if node_id == 1:
            print("this was node 1")
            print data
        elif node_id == 2:
            print("something else")
        elif node_id == 3:
            print("something else")
        elif node_id == 4:
            print("something else")
        elif node_id == 5:
            print("something else")
        elif node_id == 6:
            print("something else")
        elif node_id == 7:
            print("something else")
        else:
            print("this was node 0")
            print data
        t+=1
    else:
        t+=1

    #close port when finished
#ser.write(r"C\r") should close canbusb but doeesnt
ser.close()

