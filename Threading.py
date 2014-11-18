import socket, sys, threading #, socketserver

## SERVER

def threadFunc(conn, addr):
    while 1:
        try: #getting message
            message1 = conn.recv(2048)
            if not len(message1):
                print("Disconnected with " + addr[0] + ":" + str(addr[1]))
                conn.close()
                break
            if type(message1) == bytes:
                message1 = message1.decode("utf-8")
                print("Received: " + str(message1))
            else:
                print("Received: " + str(message1))
        except:
            print("Failed to recieve the message")
            conn.close()
            break
        if message1 == "_db_":
            if addr[0] == "127.0.0.1":
                global db_clients_INFO
                info = get_info(db_clients_INFO)
                msg1 = (str(info)).encode("utf-8")
            else:
                info = "You don\'t have permission to access this database\n"
                msg1 = (str(info)).encode("utf-8")
        elif message1 == "_info_":
            info = "1) _db_ for users info database\n"
            info += "2) _mv_ for movies database\n"
            msg1 = (str(info)).encode("utf-8")
#############################################################################################################################
#******************************************** Sensor Data Request **********************************************************#
#############################################################################################################################
        elif message1 == "Speed_data":
            #get speed data here (query sql database or get it live of the CAN)
            info = "Speed_data: "
            info += str(speed_data)
            msg1 = (str(info)).encode("utf-8")
        elif message1 == "Temp_data_all":
            #get temp data from all sources here (query sql database or get it live of the CAN)
            info = "Temp data" + str(source_id) + ": " #get the id of the node which sent the data
            info += str(temp_data)
            msg1 = (str(info)).encode("utf-8")
        elif message1 == "Bat_SOC":
            #get SOC data (query sql database or get it live of the CAN)
            info = "SOC: " 
            info += str(SOC)
            msg1 = (str(info)).encode("utf-8")
#############################################################################################################################
#******************************************************** Errors ***********************************************************#
#############################################################################################################################
        else:
            message1 = "Unknown command \nType: _info_ for a list of commands"
            msg1 = (str(message1)).encode("utf-8")
#############################################################################################################################
#**************************************************** Sending Message ******************************************************#
#############################################################################################################################
        try:
            conn.sendall(msg1)
        except:
            print("Failed to send the message")


def con_info(addr):
    host,port = addr
    try:
        results = socket.getaddrinfo(host,port,0,socket.SOCK_STREAM)
    except:
        results = []
        print("[Error] cannot get info about the client")
    return results

def get_info(db):
    info = ""
    for results in db:
        for result in results:
            info += ("_"*40)
            info += "\n"
            if result[0] == socket.AF_INET:
                info += ("Family: AF_INET")
            elif result[0] == socket.AF_INET6:
                info += ("Family: AF_INET6")
            else:
                info += ("Family:",result[0])
            info += "\n"
            if result[1] == socket.SOCK_STREAM:
                info += ("Socket Type: SOCK_STREAM")
            elif result[1] == socket.SOCK_DGRAM:
                info += ("Socket Type: SOCK_DGRAM")
            else:
                info += ("Unknown type:",result[1])
            info += "\n"
            info += ("Protocol: "+str(result[2]))
            info += "\n"
            info += ("Canonical Name: "+str(result[3]))
            info += "\n"
            info += ("Socket Address: "+", ".join([str(x) for x in result[4]]))
            info += "\n"
            info += ("_"*40)
            info += "\n"
    return info

def film_info():
    db1 = get_movie_db()
    info = ""
    info += "Not yet implemented" #
    info += "\n"
    return info
    
def get_movie_db():
    db = []
    return db


def main():
    global db_clients_INFO
    global db_clients_IP
    host = ''
    port = 51423
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((host,port))
    s.listen(5)
    print("Starting server")
    while 1:
        conn, addr = s.accept()
        c = con_info(addr)
        if c[0][4][0] not in db_clients_IP:
            db_clients_IP.append(c[0][4][0])
            db_clients_INFO.append(c)
        t = threading.Thread(target = threadFunc, args = (conn, addr))
        print("Connected with: " + addr[0] + ":" + str(addr[1]))
        t.start()
    s.close()


db_clients_INFO = []
db_clients_IP = []
main()
