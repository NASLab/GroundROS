from communication import MessageVerification, getIP

com = MessageVerification(True) # "True" for verbose
my_ip = getIP() # get IP of this machine
com.connectToServer(my_ip) # should be changed to the address of other node communicating between two machines
com.sendMessage("something") # send's a "something" message
com.verifyMessage("what now") # verify if a "what now" message has been received
com.close()
