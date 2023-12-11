#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import numpy as np
import itertools
import matplotlib.pyplot as plt
from matplotlib import animation
import cmath
import threading
import time
import csv

class ImuDataCollector:
    def __init__(self):
        rospy.init_node('imu_dtft', anonymous=True)
        self.imu_acceleration = [[], [], []]
        # self.imu_angularvelocity = [[], [], []]
        # self.imu_mag = [[], [], []]
        self.imu_acceleration_f = [[], [], []]
        # self.imu_angularvelocity_f = [[], [], []]
        # self.imu_mag_f = [[], [], []]
        self.imu_acceleration_d = [[], [], []]
        # self.imu_angularvelocity_d = [[], [], []]
        # self.imu_mag_d = [[], [], []]
        self.max_data_count = 100
        self.frequency = 100
        # self.omega = np.arange(0, self.frequency, self.frequency/self.max_data_count)
        self.omega = np.arange(0, 2*cmath.pi, 2*cmath.pi/self.max_data_count)
        # print(len(self.omega))

        self.data_enough = False

        thread1 = threading.Thread(target=self.dtft_plot)
        thread1.setDaemon(True)
        thread1.start()
        # thread2 = threading.Thread(target=self.DTFT)
        # thread2.setDaemon(True)
        # thread2.start()
        # thread3 = threading.Thread(target=self.InverseDTFT)
        # thread3.setDaemon(True)
        # thread3.start()
        # thread4 = threading.Thread(target=self.amp)
        # thread4.setDaemon(True)
        # thread4.start()
        # thread5 = threading.Thread(target=self.savecsv)
        # thread5.setDaemon(True)
        # thread5.start()
        thread6 = threading.Thread(target=self.FFT)
        thread6.setDaemon(True)
        thread6.start()

        # Subscribe to the /imu/data_raw topic
        rospy.Subscriber('/imu/data_ekf', Imu, self.imu_callback)
        # rospy.Subscriber('/imu/data_cutg', Imu, self.imu_callback)
        # rospy.Subscriber('/imu/data_raw', Imu, self.imu_callback)


    def imu_callback(self, data):
        if(len(self.imu_acceleration[0])==self.max_data_count): self.data_enough = True
        if len(self.imu_acceleration[0]) >= self.max_data_count:
            self.imu_acceleration[0].pop(0)
            self.imu_acceleration[1].pop(0)
            self.imu_acceleration[2].pop(0)

        self.imu_acceleration[0].append(data.linear_acceleration.x)
        self.imu_acceleration[1].append(data.linear_acceleration.y)
        self.imu_acceleration[2].append(data.linear_acceleration.z)
        # self.imu_acceleration[0].append(data.angular_velocity.x)
        # self.imu_acceleration[1].append(data.angular_velocity.y)
        # self.imu_acceleration[2].append(data.angular_velocity.z)
    
    # 離散時間フーリエ変換関数
    def DTFT(self):
        self.dtft = [np.zeros(self.max_data_count, dtype=np.complex)]*3
        self.ans_dtft = [np.zeros(self.max_data_count, dtype=np.complex)]*3
        self.original = self.imu_acceleration 
        s = 0
        while True:
            s+=1
            if(self.data_enough):
                for axis in range(3):
                    dtft = np.zeros(self.max_data_count, dtype = np.complex)
                    for i, f in enumerate(self.omega):
                        for n, xn_i in enumerate(self.original[axis]):
                            # dtft[i] += xn_i * cmath.exp(-1j * 2*cmath.pi* f/self.frequency * n)
                            dtft[i] += xn_i * cmath.exp(-1j * f * n)
                    # dtft[:self.max_data_count//2] *= 2
                    # dtft[self.max_data_count//2:] = 0
                    dtft[0] = 0 # cut gravity
                    # self.imu_acceleration_f[axis] = np.real(dtft)
                    self.imu_acceleration_f[axis] = np.abs(dtft)
                    # self.imu_acceleration_f[axis] = np.abs(np.real(dtft))
                    # self.imu_acceleration_f[axis] = np.abs(np.imag(dtft))
                    self.dtft[axis] = dtft
                if(s==300):
                    self.saver = np.hstack((np.array(self.imu_acceleration).T, np.array(self.dtft).T))
                    self.savecsv()
                    # self.ans_dtft[axis] = self.ans_DTFT(self.omega, self.original[axis])
            time.sleep(0.01)
    
    # あってるかチェック(合ってた)
    def ans_DTFT(self, omega, xn):
        _Xw = np.zeros(len(omega), dtype = np.complex)
        
        for i, w in enumerate(omega):
            for n, xn_i in enumerate(xn):
                _Xw[i] += xn_i * cmath.exp(-1j * w * n)
        
        _Xw[:self.max_data_count//2] *= 2
        _Xw[self.max_data_count//2:] = 0
        _Xw[0] = 0
        _Xw = np.abs(_Xw)

        return _Xw
    
    # 離散時間フーリエ逆変換計算
    # def InverseDTFT(self):
    #     # nを指定して、シンプソンの公式で逆離散時間フーリエ変換を計算
    #     def InvertWithSimpsonRule(Xomega,omega_array,n):
    #         lim_n = int(len(Xomega)/2)
    #         h = abs(omega_array[1] - omega_array[0])
            
    #         sum = np.zeros(1, dtype=complex)
    #         for i in range(1, lim_n):
    #             f1 = (Xomega[2*i - 2] * (cmath.exp((1j) * omega_array[2*i - 2] * n )) )
    #             f2 = (Xomega[2*i - 1] * (cmath.exp((1j) * omega_array[2*i - 1] * n )) )
    #             f3 = (Xomega[2*i - 0] * (cmath.exp((1j) * omega_array[2*i - 0] * n )) )
    #             sum[0] += h*(f1 + 4*f2 + f3)/3
            
    #         return sum[0] / (2 * cmath.pi)
        
    #     # omega_array = 2*cmath.pi / self.frequency * self.omega
    #     omega_array = self.omega
    #     while True:
    #         if(self.data_enough):
    #             for axis in range(3):
    #                 idtft = np.zeros(self.max_data_count, dtype=np.complex)
    #                 for n in range(self.max_data_count):
    #                     idtft[n] = InvertWithSimpsonRule(self.dtft[axis], omega_array, n)
    #                 self.imu_acceleration_d[axis] = idtft
    #                 # self.imu_acceleration_d[axis] = np.abs(idtft)
    #                 # self.imu_acceleration_d[axis] = np.abs(np.real(idtft))
    #                 # self.imu_acceleration_d[axis] = np.abs(np.imag(idtft))
    #         time.sleep(0.01)

    # nを変化させながら離散時間フーリエ逆変換計算
    def InverseDTFT(self, omega_array, Xw_array):
        n_array = np.arange(len(omega_array))
        an = np.zeros(len(n_array), dtype=np.complex)
        xn = np.zeros(len(n_array), dtype=np.complex)
        for n, n_atom in enumerate(n_array):
            lim_n = int(len(Xw_array)/2)
            h = abs(omega_array[1] - omega_array[0])
            
            sum = np.zeros(1, dtype=complex)
            for i in range(1, lim_n):
                f1 = (Xw_array[2*i - 2] * (cmath.exp((1j) * omega_array[2*i - 2] * n )) )
                f2 = (Xw_array[2*i - 1] * (cmath.exp((1j) * omega_array[2*i - 1] * n )) )
                f3 = (Xw_array[2*i - 0] * (cmath.exp((1j) * omega_array[2*i - 0] * n )) )
                sum[0] += h*(f1 + 4*f2 + f3)/3
            an[n] = sum[0] / (2 * cmath.pi)
            xn[n] = - an[n] / (2*cmath.pi*self.frequency/self.max_data_count*n)**2
            
        return xn, an

    def FFT(self):
        self.fft = [np.zeros(self.max_data_count, dtype=np.complex)]*3
        self.original = self.imu_acceleration 
        while True:
            if(self.data_enough):
                for axis in range(3):
                    fft = np.fft.fft(self.original[axis])
                    # fft[:self.max_data_count//2] *= 2
                    # fft[self.max_data_count//2:] = 0
                    fft[0] = 0 # cut gravity
                    # self.imu_acceleration_f[axis] = np.real(fft)
                    # self.imu_acceleration_f[axis] = np.abs(fft)
                    # self.imu_acceleration_f[axis] = np.abs(np.real(fft))
                    # self.imu_acceleration_f[axis] = np.abs(np.imag(fft))
                    self.fft[axis] = np.abs(fft)
            # time.sleep(0.01)
        

    def dtft_plot(self):
        fig = plt.figure(figsize=(10, 6))
        params = {
            'fig': fig,
            'func': self.update,  # グラフを更新する関数
            'interval': 10,  # 更新間隔 (ミリ秒)
            'frames': itertools.count(0, 0.1),  # フレーム番号を生成するイテレータ
        }
        anime = animation.FuncAnimation(**params)
        plt.show()

    def update(self, frame):
        # print(len(self.imu_acceleration_f[0]))
        time.sleep(2)
        if(self.data_enough):
            plt.cla()
            plt.ylim(0,100)
            # plt.ylim(-50, 50)
            plt.grid()
            self.y = np.arange(0, self.frequency, int(self.frequency/self.max_data_count))
            # print(self.y)
            # raw a
            # plt.plot(np.arange(self.max_data_count), self.imu_acceleration[0], color="red")
            # plt.plot(np.arange(self.max_data_count), self.imu_acceleration[1], color="blue")
            # plt.plot(np.arange(self.max_data_count), self.imu_acceleration[2], color="green")
            # real
            # plt.plot(np.arange(self.max_data_count), self.ret_dtft_x_real[0], color="pink") #############################################
            # plt.plot(np.arange(self.max_data_count), self.ret_dtft_real[1], color="cyan")
            # plt.plot(np.arange(self.max_data_count), self.ret_dtft_real[2], color="lime")
            # dtft answer
            # plt.plot(self.omega[:int(self.max_data_count/2)], self.ans_dtft[0][:int(self.max_data_count/2)], color="pink")
            # plt.plot(self.omega[:int(self.max_data_count/2)], self.ans_dtft[1][:int(self.max_data_count/2)], color="lime")
            # plt.plot(self.omega[:int(self.max_data_count/2)], self.ans_dtft[2][:int(self.max_data_count/2)], color="cyan")
            # imu f
            # plt.plot(self.y[:int(self.max_data_count/2)], self.imu_acceleration_f[0][:int(self.max_data_count/2)], color="red")
            # plt.plot(self.y[:int(self.max_data_count/2)], self.imu_acceleration_f[1][:int(self.max_data_count/2)], color="green")
            # plt.plot(self.y[:int(self.max_data_count/2)], self.imu_acceleration_f[2][:int(self.max_data_count/2)], color="blue")
            # fft
            plt.plot(self.y[:int(self.max_data_count/2)], self.fft[0][:int(self.max_data_count/2)], color="red")
            plt.plot(self.y[:int(self.max_data_count/2)], self.fft[1][:int(self.max_data_count/2)], color="green")
            plt.plot(self.y[:int(self.max_data_count/2)], self.fft[2][:int(self.max_data_count/2)], color="blue")
            # plt.plot(range(self.max_data_count), self.imu_acceleration[0], color="black")
            # plt.plot(range(self.max_data_count), self.self.amplitude_real[0], color="black")

    def amp(self):
        self.data_index = [[], [], []]
        self.data_value = [[], [], []]
        # self.amplitude_imag = [[], [], []]
        # self.amplitude_real = [[], [], []]
        self.ret_dtft = [[], [], []]
        self.ret_dtft_real = [[], [], []]
        self.ret_dtft_x = [[], [], []]
        self.ret_dtft_x_real = [[], [], []]
        while True:
            print("- * "*20)
            if(self.data_enough):
                for axis in range(3):
                    # 最大値のインデックスを取得
                    max_index = np.argmax(self.imu_acceleration_f[axis][:int(self.max_data_count/2)])
                    max_value = self.imu_acceleration_f[axis][max_index]

                    # 最大値の周辺のデータを抽出
                    window_size = 3  # データをいくつ抽出するか
                    ret = top_n_elements_with_indices(self.dtft[axis], window_size) # [降順でn個抜き出したデータ]
                    # 自分でidtft ------------------
                    # 振幅を求めるために変位を計算

                    self.ret_dtft_x[axis] = np.zeros(self.max_data_count, dtype = np.complex)
                    for n in range(self.max_data_count):
                        # self.ret_dtft_x[axis][n] = 0
                        for o, om in enumerate(self.omega):
                            if(o!=0):
                                self.ret_dtft_x[axis][n] += - ret[o] * cmath.exp(1j * om * n) / (2*cmath.pi) / (2*cmath.pi*o * self.frequency/self.max_data_count)**2
                        # a = np.array([ for o, om in enumerate(self.omega)])
                    self.ret_dtft_x_real[axis] = 1000* np.real(self.ret_dtft_x[axis])

                    # ------------------------------

                    # self.ret_dtft[axis], self.ret_dtft_x[axis] = self.InverseDTFT(self.omega, ret)
                    # self.ret_dtft_real[axis] = np.real(self.ret_dtft[axis])
                    # self.ret_dtft_x_real[axis] = 1000 * np.real(self.ret_dtft_x[axis])
                    # start_index = max(1, max_index - window_size)
                    # end_index = min(len(self.imu_acceleration_f[axis]), max_index + window_size + 1)
                    # self.data_index[axis] = np.arange(start_index, end_index)
                    # self.data_value[axis] = self.dtft[axis][start_index:end_index]
                    # self.amplitude_imag[axis] = np.array([self.data_value[axis][n] * cmath.exp(1j * self.omega[self.data_index[axis][n]] * self.data_index[axis][n]) / (2*cmath.pi*(2*cmath.pi*self.frequency/self.max_data_count*self.data_index[axis][n])**2) for n in range(len(self.data_index[axis]))])
                    # self.amplitude_imag[axis] = np.array([np.abs(self.data_value[axis][n]) * cmath.exp(1j * self.omega[self.data_index[axis][n]] * self.data_index[axis][n]) / (2*cmath.pi*(2*cmath.pi*self.frequency/self.max_data_count*self.data_index[axis][n])**2) for n in range(len(self.data_index[axis]))]) # absとして計算
                    # self.amplitude_real[axis] = np.sum(np.array([np.abs(self.amplitude_imag[axis][n])*(np.cos(cmath.phase(self.amplitude_imag[axis][n]))+np.sin(cmath.phase(self.amplitude_imag[axis][n]))) for n in range(len(self.amplitude_imag[axis]))]))
                    # self.amplitude_real[axis] = np.sum(np.array([np.abs(np.real(self.amplitude_imag[axis][n])) for n in range(len(self.amplitude_imag[axis]))])) # 虚部を無視
                    # self.amplitude_real[axis] = np.sum(np.array([np.real(self.amplitude_imag[axis][n]) for n in range(len(self.amplitude_imag[axis]))])) # 虚部を無視
                    # if(axis==0):
                    #     print(f"index: {self.data_index[axis]}")
                    #     print(f"value: {self.data_value[axis]}")
                    #     # print("- * "*5)
                    #     # print(f"amp: {self.amplitude_imag[axis]}")
                    #     # print(f"amp real: {self.amplitude_real[axis]}")
                    #     # print(f"最大の要素番号とomega : {self.data_index[axis][0]}, {self.omega[self.data_index[axis][0]]}")
                    #     print(f"最大値: {self.data_value[axis][0]}")
                    #     print(f"最大値の偏角: {np.degrees(cmath.phase(self.data_value[axis][0])):.3f}")
                    #     print(f"振幅の複素数: {self.amplitude_imag[axis]}")
                    #     print(f"振幅の実数: {self.amplitude_real[axis]}")
                    #     print(f"振幅 : {np.abs(self.amplitude_real[axis]*1000):.3f} [mm]")
            time.sleep(0.1)
    
    def savecsv(self):
        saver = self.saver
        # CSVファイルに書き込み
        filename = 'output.csv'
        with open(filename, 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerows(saver)
            # while True:
            #     csvwriter.writerows(self.saver)
            #     # csvwriter.writerow(self.imu_acceleration[0][:5])
            #     time.sleep(0.1)
            print("save")

def top_n_elements_with_indices(arr, n, value=False):
    # 配列のインデックスと値を組み合わせたタプルのリストを作成
    indexed_array = list(enumerate(arr))

    # 値に基づいて降順にソート
    sorted_array = sorted(indexed_array, key=lambda x: x[1], reverse=True)

    # 上位n個の要素とその対応するインデックスを抜き出す
    top_n_elements = sorted_array[:n]

    if(value): return top_n_elements
    else:
        # 全体を0で初期化
        result_array = np.zeros_like(arr)

        # 上位n個の要素のインデックスと値を新しい配列にコピー
        for idx, value in top_n_elements:
            result_array[idx] = value

        # 結果を返す
        return result_array
    

if __name__ == '__main__':
    imu_dtft = ImuDataCollector()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting down")