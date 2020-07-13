import matplotlib.pyplot as plt
from inverted_pendulum import InvertedPendulum
import numpy as np
import math
import timeit

x0 = 2
y0 = 0.5
z0 = 0.8
x1 = 0.001
y1 = 0.001
end_time = 0.5
dt = 0.001

pendulum = InvertedPendulum()
pendulum.taylor3_to_t(x0, y0, z0, x1, y1, 1)

# print(timeit.timeit('InvertedPendulum.taylor3_to_t(0.001, 0.001, 1, 0.03, 0, 0.5)',
#                     number = 1000,
#                     setup='from inverted_pendulum import InvertedPendulum'))
# print(timeit.timeit('InvertedPendulum.taylor_to_t(5, 0.001, 0.001, 1, 0.03, 0, 0.5, 0.001)',
#                     number = 1000,
#                     setup='from inverted_pendulum import InvertedPendulum'))
# print(timeit.timeit('InvertedPendulum.numeric_solve_to_t(0.001, 0.001, 1, 0.03, 0, 0.5, 0.001)',
#                     number = 1000,
#                     setup='from inverted_pendulum import InvertedPendulum'))


print("num deriv o2 = ", pendulum.numeric_derivative(2, x0, y0, z0, x1, y1, dt))
print("num deriv o3 = ", pendulum.numeric_derivative(3, x0, y0, z0, x1, y1, dt))
# print("num deriv o4 = ", pendulum.numeric_derivative(4, x0, y0, z0, x1, y1, dt))
# print("num deriv o5 = ", pendulum.numeric_derivative(5, x0, y0, z0, x1, y1, dt))
# print("num deriv o6 = ", pendulum.numeric_derivative(6, x0, y0, z0, x1, y1, dt))
# print("num deriv o7 = ", pendulum.numeric_derivative(7, x0, y0, z0, x1, y1, dt))

plot = 0
if plot:
    times = np.linspace(0, end_time, end_time * 1000)
    num_taylor_x = []
    num_taylor_y = []
    num_taylor_z = []
    num_taylor_theta = []
    for t in times:
        sol = pendulum.taylor_to_t(3, x0, y0, z0, x1, y1, t, dt)
        num_taylor_x.append(sol['x'])
        num_taylor_y.append(sol['y'])
        num_taylor_z.append(sol['z'])
        num_taylor_theta.append(math.atan2(sol['z'], math.sqrt(sol['x']**2 + sol['y']**2)))


    taylor3_x = []
    taylor3_y = []
    taylor3_z = []
    taylor3_theta = []
    for t in times:
        sol = pendulum.taylor3_to_t(x0, y0, z0, x1, y1, t)
        taylor3_x.append(sol['x'])
        taylor3_y.append(sol['y'])
        taylor3_z.append(sol['z'])
        taylor3_theta.append(math.atan2(sol['z'], math.sqrt(sol['x']**2 + sol['y']**2)))

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
    dt = 0.001
    t = 0
    while t < end_time:
        t = t + dt
        x, y, z, vx, vy = pendulum.step_numeric_solve(x, y, z, vx, vy, dt)
        num_t.append(t)
        num_x.append(x)
        num_y.append(y)
        num_z.append(z)
        num_theta.append(math.atan2(z, math.sqrt(x**2 + y**2)))

    error = []
    for i in range(int(end_time / dt)):
        error.append(num_theta[i] - num_taylor_theta[i])

    # plt.plot(times, num_taylor_x)
    # # plt.plot(times, num_taylor_y)
    # plt.plot(times, num_taylor_z)
    plt.plot(times, num_taylor_theta)
    # # plt.plot(times, taylor3_x)
    # # plt.plot(times, taylor3_y)
    # # plt.plot(times, taylor3_z)
    plt.plot(times, taylor3_theta)
    # # # plt.plot(num_t, num_x)
    # # # plt.plot(num_t, num_y)
    # # # plt.plot(num_t, num_z)
    plt.plot(num_t, num_theta)
    # plt.plot(times, error)
    # plt.legend(['taylor x', 'taylor y', 'taylor z', 'taylor theta', 'num x', 'num y', 'num z', 'num theta'])
    plt.legend(['num taylor 5 theta', 'taylor 3 theta', 'num theta, dt = 0.001', 'num theta, dt = 0.00001'])
    plt.ylabel('Distance (m)')
    plt.xlabel('Time (s)')
    plt.show()

