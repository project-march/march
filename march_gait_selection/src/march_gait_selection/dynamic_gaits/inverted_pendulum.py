import math


class InvertedPendulum(object):
    G = 9.81
    EPSILON = 0.000001

    @classmethod
    def numeric_solve_to_t(cls, x, y, z, vx, vy, t, dt=0.01):
        time = 0
        while time < t:
            x, y, z, vx, vy = cls.step_numeric_solve(x, y, z, vx, vy, dt)
            time += dt
        if abs(z) < cls.EPSILON:
            vz = 0
        else:
            vz = - (x * vx + y * vy) / z
        return {'x': x, 'y': y, 'z': z, 'vx': vx, 'vy': vy, 'vz': vz}

    @classmethod
    def step_numeric_solve(cls, x0, y0, z0, vx0, vy0, dt=0.01):
        r = math.sqrt(x0**2 + y0**2 + z0**2)
        if abs(z0) < cls.EPSILON:
            vz0 = 0
        else:
            vz0 = - (x0 * vx0 + y0 * vy0) / z0
        vsquared = vx0**2 + vy0**2 + vz0**2

        ax = x0 * (cls.G * z0 - vsquared) / r**2
        ay = y0 * (cls.G * z0 - vsquared) / r**2

        vx1 = vx0 + dt * ax
        vy1 = vy0 + dt * ay

        x1 = x0 + dt * 0.5 * (vx0 + vx1)
        y1 = y0 + dt * 0.5 * (vy0 + vy1)
        z1 = math.sqrt(max(r**2 - x1**2 - y1**2, 0))

        return x1, y1, z1, vx1, vy1
