import PID as pid

m=pid.PID(1,1,1)
print m.calculate(0)