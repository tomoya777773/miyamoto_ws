import rospy

import numpy as np
import time
import socket


HOST = '10.42.0.175'
PORT = 30001

# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.connect((HOST,PORT))
# s.send("set_digital_out(2,False)" + "\n")
# data = s.recv(1024)
# s.close()
# print "Received" + repr(data)

print "Starting Program"
count = 0

while (count < 1):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    time.sleep(0.5)
    print "Set output 1 and 2 high"
    s.send ("set_digital_out(1,True)" + "\n")
    time.sleep(0.1)

    s.send ("set_digital_out(2,True)" + "\n")
    time.sleep(2)
    print "Robot starts Moving to 3 positions based on joint positions"
    t1 = time.time()

    s.send ("movel([-2.1155007521,-1.550907437,-2.2655852477,-0.9283059279,1.5687683821,-1.2148779074], a=8.0, v=1.0)" + "\n")
    # s.send ("movel([-2.0909293334,-1.4590304534,-2.3394396941,-0.947268788,1.5694642067,-1.1900661627], a=8.0, v=1.0)" + "\n")
    
    t2 = time.time()

    print t2 - t1

    
    time.sleep(10)

    print "Set output 1 and 2 low"
    s.send ("set_digital_out(1,False)" + "\n")
    time.sleep(0.1)

    s.send ("set_digital_out(2,False)" + "\n")
    time.sleep(0.1)
    count = count + 1
    print "The count is:", count
    print "Program finish"
    time.sleep(1)
    data = s.recv(1024)
    s.close()
    print ("Received", repr(data))

print "Status data received from robot"




# print "Starting Program"
# count = 0
 
# while (count < 1000):
#     s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#     s.bind((HOST, PORT)) # Bind to the port 
#     s.listen(5) # Now wait for client connection.
#     c, addr = s.accept() # Establish connection with client.
#     try:
#         msg = c.recv(1024)
#         print msg
#         time.sleep(1)
#         if msg == "asking_for_data":
#             count = count + 1
#             print "The count is:", count
#             time.sleep(0.5)
#             print ""
#             time.sleep(0.5)

#             c.send("(-2.1155007521,-1.550907437,-2.2655852477,-0.9283059279,1.5687683821,-1.2148779074)");
#             print "Send -2.1155007521,-1.550907437,-2.2655852477,-0.9283059279,1.5687683821,-1.2148779074"
        
#     except socket.error as socketerror:
#         print count
 
# c.close()
# s.close()