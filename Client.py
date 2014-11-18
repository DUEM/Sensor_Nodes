









Open with


















































import socket, sys

## CLIENT

t = True
host = "localhost"
port = 51423
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    sock.connect((host,port)) # sock.connec_ex((host,port)) !!!
except:
    print("Something is wrong with connecting to the server at %d." % (port))
    sock.close()
    t = False

while t:
    try:
        message1 = str(input("Input: "))
        if message1.lower() == "_exit_":
            t = False
        sock.sendall(message1.encode("utf-8"))
    except:
        print("Failed to send the message")

    try:
        msg1 = sock.recv(2048)
        if len(msg1):
            msg2 = msg1.decode("utf-8")
            print(str(msg2))
    except:
        print("Failed to recieve the message")
sock.close()
    

