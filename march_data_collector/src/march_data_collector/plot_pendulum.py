import matplotlib.pyplot as plt
from inverted_pendulum import InvertedPendulum
import numpy as np
import math
import timeit

x0 = -0.03928103222568591
y0 = 0.23100714329053423
z0 = 0.6747309776542834
x1 = 0.000
y1 = 0.000
end_time = 2.7
dt = 0.001

pendulum = InvertedPendulum()

print(pendulum.numeric_solve_to_t(x0, y0, z0, x1, y1, end_time, 0.01))
print(pendulum.calculate_falling_time(x0, y0, z0, x1, y1, 0.01))

# print(timeit.timeit('InvertedPendulum.taylor3_to_t(0.001, 0.001, 1, 0.03, 0, 0.5)',
#                     number = 1000,
#                     setup='from inverted_pendulum import InvertedPendulum'))
# print(timeit.timeit('InvertedPendulum.taylor_to_t(5, 0.001, 0.001, 1, 0.03, 0, 0.5, 0.001)',
#                     number = 1000,
#                     setup='from inverted_pendulum import InvertedPendulum'))
# print(timeit.timeit('InvertedPendulum.numeric_solve_to_t(0.001, 0.001, 1, 0.03, 0, 0.5, 0.001)',
#                     number = 1000,
#                     setup='from inverted_pendulum import InvertedPendulum'))


# print("num deriv o2 = ", pendulum.numeric_derivative(2, x0, y0, z0, x1, y1, dt))
# print("num deriv o3 = ", pendulum.numeric_derivative(3, x0, y0, z0, x1, y1, dt))
# print("num deriv o4 = ", pendulum.numeric_derivative(4, x0, y0, z0, x1, y1, dt))
# print("num deriv o5 = ", pendulum.numeric_derivative(5, x0, y0, z0, x1, y1, dt))
# print("num deriv o6 = ", pendulum.numeric_derivative(6, x0, y0, z0, x1, y1, dt))
# print("num deriv o7 = ", pendulum.numeric_derivative(7, x0, y0, z0, x1, y1, dt))

plot = 1
if plot:
    num_t = [0]
    num_x = [x0]
    num_y = [y0]
    num_z = [z0]
    num_theta = [math.atan2(z0, math.sqrt(x0**2 + y0**2))]
    x = x0
    y = y0
    z = z0
    vx = x1
    vy = y1
    dt = 0.01
    t = 0
    while t < end_time:
        t = t + dt
        x, y, z, vx, vy = pendulum.step_numeric_solve(x, y, z, vx, vy, dt)
        num_t.append(t)
        num_x.append(x)
        num_y.append(y)
        num_z.append(z)
        num_theta.append(math.atan2(z, math.sqrt(x**2 + y**2)))

    num_t2 = [0]
    num_x2 = [x0]
    num_y2 = [y0]
    num_z2 = [z0]
    num_theta2 = [math.atan2(z0, math.sqrt(x0**2 + y0**2))]
    x = x0
    y = y0
    z = z0
    vx = x1
    vy = y1
    dt = 0.0001
    t = 0
    while t < end_time:
        t = t + dt
        x, y, z, vx, vy = pendulum.step_numeric_solve(x, y, z, vx, vy, dt)
        num_t2.append(t)
        num_x2.append(x)
        num_y2.append(y)
        num_z2.append(z)
        num_theta2.append(math.atan2(z, math.sqrt(x**2 + y**2)))

    # error = []
    # for i in range(int(end_time / dt)):
    #     error.append(num_theta[i] - num_taylor_theta[i])

    plt.plot(num_t, num_x)
    plt.plot(num_t, num_y)
    plt.plot(num_t, num_z)
    plt.plot(num_t, num_theta)
    plt.plot(num_t2, num_x2)
    plt.plot(num_t2, num_y2)
    plt.plot(num_t2, num_z2)
    plt.plot(num_t2, num_theta2)
    # plt.plot(times, error)
    # plt.legend(['taylor x', 'taylor y', 'taylor z', 'taylor theta', 'num x', 'num y', 'num z', 'num theta'])
    plt.legend(['num x, dt = 0.001', 'num y, dt = 0.001', 'num z, dt = 0.001', 'num theta, dt = 0.001',
                'num x, dt = 0.0001', 'num y, dt = 0.0001', 'num z, dt = 0.0001', 'num theta, dt = 0.0001'])
    plt.ylabel('Distance (m)')
    plt.xlabel('Time (s)')
    plt.show()