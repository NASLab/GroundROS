from communication import MessageVerification, getIP

com = MessageVerification(True) # "True" for verbose
my_ip = getIP() # get IP of this machine
print my_ip
com.connectToServer('192.168.4.11')# should be changed to the address of other node communicating between two machines
# com.verifyMessage("something") # verify if a "something" message has been received
com.sendMessage("start mission") # send's a "what now" message
com.close()
