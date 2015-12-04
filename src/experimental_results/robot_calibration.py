from scipy.optimize import minimize, fmin_cobyla, fmin_tnc, fmin_slsqp, fmin_l_bfgs_b
from path_planning_analysis_cost_function import calibration_cost_function
from scipy.optimize import differential_evolution
x0 = [0.23, -0.08]

# print minimize(calibration_cost_function,x0,  method='Nelder-Mead')


def c1(x):
    # x0<.4
    return .4-x[0]


def c2(x):
    # x0>0
    return x[0]


def c3(x):
    # x1>-.1
    return x[1] + .1


def c4(x):
    # x1<.1
    return .1 - x[1]
    # return [c1,c2,c3,c4]

cons = ({'type': 'ineq', 'fun': lambda x: x[0] + .4},
        {'type': 'ineq', 'fun': lambda x: .1 - x[0]},
        {'type': 'ineq', 'fun': lambda x: x[1] + .2},
        {'type': 'ineq', 'fun': lambda x: .2 - x[1]},)


# bfgs_options = {'epa':.01}
# print minimize(calibration_cost_function,x0,method = 'L-BFGS-B')

# cobyla_options = {'rhobeg':.005}
# print minimize(calibration_cost_function,x0,constraints = cons,options =
# cobyla_options)


print fmin_cobyla(calibration_cost_function, x0, [c1, c2, c3, c4],
rhobeg=.01)


b = [(0, .3), (-.1, .1)]
print differential_evolution(calibration_cost_function, b)

print fmin_tnc(calibration_cost_function, x0, approx_grad=True, bounds=b, epsilon=.01)


print fmin_slsqp(calibration_cost_function, x0, bounds=b, epsilon=.01)
