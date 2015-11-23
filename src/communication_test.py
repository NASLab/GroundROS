#!/usr/bin/env python

from sys import path
path.append('Modules/')
# import communication

from communication import LabNavigation

# print communication

com = LabNavigation()

while True:
    print '-------------------------------------'
    print 0,com.getStates(0)
    print 1,com.getStates(1)
com.close()