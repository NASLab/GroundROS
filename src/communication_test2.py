#!/usr/bin/env python

from sys import path
path.append('Modules/')
# import communication

from communication import LabNavigation

# print communication

com = LabNavigation()

while True:
    print '-------------------------------------'
    print 2,com.getStates(2)
    print 3,com.getStates(3)
com.close()