import pandas as pd
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 楕円球の最小二乗法フィッティング
def fitting_ellipse(x,y,z):
    [x, y, z] = [np.array(x), np.array(y), np.array(z)]
    m = np.array([x**2, y**2, z**2, x, y, z])
    # m = np.array([x**2, y**2, z**2, 2*x*y, 2*y*z, 2*x*z, x, y, z])
    M = np.vstack([np.sum(m*i, axis=1) for i in m])
    M_inv = np.linalg.inv(M)
    m_sum = np.sum(m, axis=1)
    # [a11, a22, a33, a12, a23, a13, b1, b2, b3] = -M_inv @ m_sum.T
    [a11, a22, a33, b1, b2, b3] = -M_inv @ m_sum.T
    # print([a11, a22, a33, b1, b2, b3])
    center = [b1/(2*a11), b2/(2*a22), b3/(2*a33)]
    offset = - (b1**2/(4*a11) + b2**2/(4*a22) + b3**2/(4*a33) + 1)
    [a, b, c] = [(offset/a11)**0.5, (offset/a22)**0.5, (offset/a33)**0.5]
    # print(f"center: {center}")
    # print(f"offset: {offset}")
    # print(f"param: {a, b, c}")
    return a, b, c, center[0], center[1], center[2]

def main():
    # CSVファイルのパス
    csv_file_path = "/home/sskr3/catkin_ws/src/scanner/data_csv/for_calibraiton/imu_data_2023-11-20_23-3.csv"
    # mag_path = "/home/sskr3/catkin_ws/src/scanner/data_csv/for_calibraiton/imu_data_20231120-1911.csv"

    # CSVファイルをpandas DataFrameに読み込む
    df = pd.read_csv(csv_file_path)
    # df = pd.read_csv(mag_path)
    accel = [np.ravel([df.iloc[:, 0]]), np.ravel([df.iloc[:, 1]]), np.ravel([df.iloc[:, 2]])]
    omega = [np.ravel([df.iloc[:, 3]]), np.ravel([df.iloc[:, 4]]), np.ravel([df.iloc[:, 5]])]

    a, b, c, p, q, r = fitting_ellipse(accel[0], accel[1], accel[2])

    print("- * - data infomation - * -")
    # print(f"acceleration_x mean: {np.mean(accel[0])}")
    # print(f"acceleration_y mean: {np.mean(accel[1])}")
    # print(f"acceleration_z mean: {np.mean(accel[2])}")
    print("acceleration: ")
    print(f"a: {a}, b:{b}, c:{c}")
    print(f"e0: {p, q, r}")
    print(f"e1: {a/9.80665-1, b/9.80665-1, c/9.80665-1}")
    print("angular velocity: ")
    print(f"e0: {np.mean(omega[0]), np.mean(omega[1]), np.mean(omega[2])}")
    print("- * " * 5)

    # 3Dプロット
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 楕円
    xy_x = a * np.cos(np.linspace(0, 2*np.pi, 129)) + p
    xy_y = b * np.sin(np.linspace(0, 2*np.pi, 129)) + q
    yz_y = b * np.cos(np.linspace(0, 2*np.pi, 129)) + q
    yz_z = c * np.sin(np.linspace(0, 2*np.pi, 129)) + r
    zx_z = c * np.cos(np.linspace(0, 2*np.pi, 129)) + r
    zx_x = a * np.sin(np.linspace(0, 2*np.pi, 129)) + p

    # ax.scatter(omega[0], omega[1], omega[2])
    ax.plot(xy_x, xy_y, np.zeros(len(xy_x)), color="red")
    ax.plot(np.zeros(len(yz_y)), yz_y, yz_z, color="blue")
    ax.plot(zx_x, np.zeros(len(zx_z)), zx_z, color="green")
    ax.scatter(accel[0], accel[1], accel[2], color="black")
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_xlim([-20, 20])
    ax.set_ylim([-20, 20])
    ax.set_zlim([-20, 20])
    ax.set_title('3D Acceleration Plot')

    plt.show()

if __name__ == "__main__":
    main()