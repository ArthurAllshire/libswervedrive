from networktables import NetworkTables
import time
import matplotlib.pyplot as plt
import math

def plot_live_nt_modules(sd, rate, names):
    plt.ion()
    plt.show()

    angles = [sd.getAutoUpdateValue(n+"_angle", 0) for n in names]
    speeds = [sd.getAutoUpdateValue(n+"_speed", 0) for n in names]
    alphas = [sd.getAutoUpdateValue(n+"_alpha", 0) for n in names]
    ls = [sd.getAutoUpdateValue(n+"_l", 0) for n in names]

    plt.xlim([-1.5, 1.5])
    plt.ylim([-1.5, 1.5])

    while True:
        t = time.time()
        X, Y, U, V = [], [], [], []
        plt.cla()
        plt.xlim([-1.5, 1.5])
        plt.ylim([-1.5, 1.5])
        for angle, speed, alpha, l in zip(angles, speeds, alphas, ls):
            X.append(l.value * math.cos(alpha.value))
            Y.append(l.value * math.sin(alpha.value))
            U.append(speed.value * math.cos(angle.value))
            V.append(speed.value * math.sin(angle.value))
        plt.quiver(X, Y, U, V)
        plt.draw()
        plt.pause(0.00000000001)
        time.sleep(0.1)

if __name__ == "__main__":
    ip = 'localhost'
    NetworkTables.initialize(server=ip)
    sd = NetworkTables.getTable("SmartDashboard")
    names = ["lr", "rr", "lf", "rf"]
    
    plot_live_nt_modules(sd, 5, names)