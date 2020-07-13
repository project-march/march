import math
import scipy.special

class InvertedPendulum(object):
    g = 9.81

    @classmethod
    def taylor2_to_t(cls, x0, y0, z0, x1, y1, t):
        r = math.sqrt(x0**2 + y0**2 + z0**2)
        z1 = - (x0 * x1 + y0 * y1) / z0
        a2 = x0 * (cls.g * z0 - 2 * z1**2 - x1**2 - y1**2) / (2 * r**2)
        b2 = y0 * a2 / x0

        x = x0 + x1 * t + a2 * t**2
        y = y0 + y1 * t + b2 * t**2
        z = math.sqrt(max(r**2 - x**2 - y**2, 0))
        vx = x1 + 2 * a2 * t
        vy = y1 + 2 * b2 * t
        if z < 0.000001:
            vz = 0
        else:
            vz = - (x * vx + y * vy) / z
        return {'x': x, 'y': y, 'z': z, 'vx': vx, 'vy': vy, 'vz': vz}

    @classmethod
    def taylor3_to_t(cls, x0, y0, z0, x1, y1, t):
        r = math.sqrt(x0**2 + y0**2 + z0**2)
        z1 = - (x0 * x1 + y0 * y1) / z0
        a2 = x0 * (cls.g * z0 - 2 * z1**2 - x1**2 - y1**2) / (2 * r**2)
        if a2 < 0.000001:
            b2 = 0
        else:
            b2 = y0 * a2 / x0

        z2 = - (2 * z1**2 + x1**2 + y1**2 + 2 * x0 * a2 + 2 * y0 * b2) / (2 * z0)
        a3 = - z0 * (z1 * a2 / 3 + \
                     ((x0 * y1 - x1 * y0) * y0 / (z0**2) - x1) * (cls.g + 2 * z2) / 6 - \
                     x0 * (z1**3 / (z0**2) - z1 * z2 / z0 - (2 * x1 * a2 + 2 * y1 * b2) / (2 * z0))) / (r**2)
        b3 = y0 * a3 / x0 + (y1 - y0 * x1 / x0) * (cls.g + 2 * z2) / (6 * z0)

        # print("a2 = ", a2)
        # print("2a2 = ", 2 * a2)
        # print("a3 = ", a3)
        # print("6a3 = ", 6 * a3)
        # print("b2 = ", b2)
        # print("2b2 = ", 2 * b2)
        # print("b3 = ", b3)
        # print("6b3 = ", 6 * b3)

        x = x0 + x1 * t + a2 * t**2 + a3 * t**3
        y = y0 + y1 * t + b2 * t**2 + b3 * t**3
        z = math.sqrt(max(r**2 - x**2 - y**2, 0))
        vx = x1 + 2 * a2 * t + 3 * a3 * t**2
        vy = y1 + 2 * b2 * t + 3 * b3 * t**2
        if z < 0.000001:
            vz = 0
        else:
            vz = - (x * vx + y * vy) / z
        return {'x': x, 'y': y, 'z': z, 'vx': vx, 'vy': vy, 'vz': vz}

    @classmethod
    def taylor_to_t(cls, order, x0, y0, z0, x1, y1, t, dt = 0.0005):
        r = math.sqrt(x0**2 + y0**2 + z0**2)
        x = x0 + x1 * t
        y = y0 + y1 * t
        vx = x1
        vy = y1
        for o in range(2, order+1):
            derivative_x, derivative_y = cls.numeric_derivative(o, x0, y0, z0, x1, y1)
            x = x + derivative_x * t**o / math.factorial(o)
            y = y + derivative_y * t**o / math.factorial(o)
            vx = vx + o * derivative_x * t**(o-1) / math.factorial(o)
            vy = vy + o * derivative_y * t**(o-1) / math.factorial(o)

        z = math.sqrt(max(r**2 - x**2 - y**2, 0))
        if z < 0.000001:
            vz = 0
        else:
            vz = - (x * vx + y * vy) / z
        return {'x': x, 'y': y, 'z': z, 'vx': vx, 'vy': vy, 'vz': vz}

    @classmethod
    def numeric_solve_to_t(cls, x0, y0, z0, x1, y1, t, dt=0.0001):
        r = math.sqrt(x0**2 + y0**2 + z0**2)
        x = x0
        y = y0
        z = z0
        vx = x1
        vy = y1
        time = 0
        while time < t:
            x, y, z, vx, vy = cls.step_numeric_solve(x, y, z, vx, vy, dt)
            time += dt
        vz = - (x * vx + y * vy) / z
        return {'x': x, 'y': y, 'z': z, 'vx': vx, 'vy': vy, 'vz': vz}

    @classmethod
    def step_numeric_solve(cls, x0, y0, z0, vx0, vy0, dt=0.0001):
        r = math.sqrt(x0**2 + y0**2 + z0**2)
        if z0 < 0.000001:
            vz0 = 0
        else:
            vz0 = - (x0 * vx0 + y0 * vy0) / z0
        v = math.sqrt(vx0**2 + vy0**2 + vz0**2)

        ax = x0 * (cls.g * z0 - v)
        ay = y0 * (cls.g * z0 - v)

        vx1 = vx0 + dt * ax
        vy1 = vy0 + dt * ay

        x1 = x0 + dt * 0.5 * (vx0 + vx1)
        y1 = y0 + dt * 0.5 * (vy0 + vy1)
        z1 = math.sqrt(max(r**2 - x1**2 - y1**2, 0))

        return x1, y1, z1, vx1, vy1

    @classmethod
    def numeric_derivative(cls, order, x0, y0, z0, vx0, vy0, dt=0.0001):
        r = math.sqrt(x0**2 + y0**2 + z0**2)
        ddotx = []
        ddoty = []
        for i in range(order - 1):
            if z0 < 0.000001:
                vz0 = 0
            else:
                vz0 = - (x0 * vx0 + y0 * vy0) / z0
            v = math.sqrt(vx0**2 + vy0**2 + vz0**2)

            ax = x0 * (cls.g * z0 - v**2) / r**2
            ay = y0 * (cls.g * z0 - v**2) / r**2
            ddotx.append(ax)
            ddoty.append(ay)

            vx1 = vx0 + dt * ax
            vy1 = vy0 + dt * ay

            x0 = x0 + dt * 0.5 * (vx0 + vx1)
            y0 = y0 + dt * 0.5 * (vy0 + vy1)
            z0 = math.sqrt(max(r**2 - x0**2 - y0**2, 0))
            vx0 = vx1
            vy0 = vy1

        # print(ddotx)

        derivative_x = 0
        derivative_y = 0
        n = order - 2
        for i in range(n + 1):
            derivative_x += (-1) ** (n - i) * scipy.special.binom(n, i) * ddotx[i]
            derivative_y += (-1) ** (n - i) * scipy.special.binom(n, i) * ddoty[i]


        return derivative_x / dt**n, derivative_y / dt**n


