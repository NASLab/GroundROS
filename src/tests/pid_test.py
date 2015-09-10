import sys
sys.path.insert(0, '..')
import PID as pid

m = pid.PID(1, 1, 1)
print str(m)
print m.calculate(0)
print m.calculate(1)
print m.calculate(2)
print m.calculate(3)
print m.calculate(4)
print m.calculate(5)
print m.calculate(6)
