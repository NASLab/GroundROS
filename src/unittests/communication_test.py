#!/usr/bin/env python

from sys import path
path.append('../Modules/')
# import communication

from communication import NaslabNetwork

# print communication

com = NaslabNetwork()

While True:
    print com.getStates()
