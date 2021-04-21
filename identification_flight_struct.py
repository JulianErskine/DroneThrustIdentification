# /usr/bin/python3
import numpy as np
import pandas as pd
import os
import sys
import datetime
import math
import matplotlib.pyplot as plt

from scipy import signal

# Base class for handling extraction and processing of data


class PX4DroneIdentificationData:
    # class constructor
    def __init__(self, flight_data_path, mass):
        # Define attributes
        self.mass = mass
        self.time = []  # Time list
        self.battery = pd.DataFrame()
        self.attitude = pd.DataFrame()
        self.imu = pd.DataFrame()
        self.control = pd.DataFrame()
        self.pwm = pd.DataFrame()

        self.ParseCSV(flight_data_path)

        list_of_data = [self.battery, self.attitude,
                        self.imu, self.control, self.pwm]

        self.list_of_data_names = ["battery",
                                   "attitude", "imu", "control", "pwm"]

        resampled_data = self.ResampleData(list_of_data)
        print("Data Re-Sampled")
        # for data in resampled_data:
        #   print("RESAMPLED: ",data)

        filtered_data = self.FilterData(resampled_data)
        print("Data Filtered")

        cut_data = self.CutData(filtered_data)
        print("Data Cut")

        self.battery = cut_data[0]
        self.attitude = cut_data[1]
        self.imu = cut_data[2]
        self.control = cut_data[3]
        self.pwm = cut_data[4]

        self.thrust = []
        self.thrust_cmd = []
        self.CalculateThrust()

        self.voltage = np.round(
            np.array(self.battery['voltage_filtered_v'].values), 4)

    # Function to extract raw data from csv
    def ParseCSV(self, flight_data_path):
        # Find log files        
        files = os.listdir(flight_data_path)
        print("\n Files in path: ", files)

        # Topic list:
        #   sensor_combined:  acceleration, angular velocity
        #   actuator_control: angular velocity and thrust setpoints
        #   actuator_outputs: motor pwm signal
        #   vehical_attitude: estimated attitude from ekf
        #   battery_status: baterry discharge rate and voltage
        topics_of_interest = ["sensor_combined", "actuator_control",
                              "actuator_outputs", "vehicle_attitude", "battery_status"]

        # remove extra .csv files
        kept_files = []
        for file in files:
            if any(substring in file for substring in topics_of_interest) and "_0" in file:
                kept_files.append(file)
            else:
                pass
        files = kept_files
        print("\n Only files : ", kept_files, " are of interest \n ")

        # Extract csv data to pandas time series
        print("Extracting CSV to DataFrame structure")
        for file in files:
            if "sensor" in file:
                print("Found IMU topic")
                self.imu = pd.read_csv(flight_data_path+file, index_col='timestamp', usecols=[
                                       "timestamp", "gyro_rad[0]", "gyro_rad[1]", "gyro_rad[2]", "accelerometer_m_s2[0]", "accelerometer_m_s2[1]", "accelerometer_m_s2[2]"])
            elif "control" in file:
                print("Found Control Setpoint topic")
                self.control = pd.read_csv(flight_data_path+file, index_col='timestamp', usecols=[
                                           "timestamp", "control[0]", "control[1]", "control[2]", "control[3]"])
                print("Raw thrust range: ", [
                      min(self.control["control[3]"]),  max(self.control["control[3]"])])
            elif "outputs" in file:
                print("Found PWM topic")
                self.pwm = pd.read_csv(flight_data_path+file, index_col='timestamp', usecols=[
                                       "timestamp", "output[0]", "output[1]", "output[2]", "output[3]", "output[4]", "output[5]"])
            elif "attitude" in file:
                print("Found Attitude topic")
                self.attitude = pd.read_csv(flight_data_path+file, index_col='timestamp', usecols=[
                                            "timestamp", "q[0]", "q[1]", "q[2]", "q[3]"])
            elif "battery" in file:
                print("Found Battery topic")
                self.battery = pd.read_csv(flight_data_path+file, index_col='timestamp', usecols=[
                                           "timestamp", "voltage_filtered_v", "current_filtered_a"])
            else:
                pass
        return  # End of ParseCSV

    # Function to resample data

    def ResampleData(self, data_series):
        resampled_data = []
        print("Rescaling data")
        t_start = math.inf
        for data in data_series:
            if data.index.min() < t_start:
                t_start = data.index.min()

        for data in data_series:
            data.index = pd.to_datetime((data.index - t_start)/1E6, unit='s')
            idx_list = data.index.tolist()
            idx_list[0] = pd.to_datetime(0, unit='s')
            data.index = idx_list
            data = data.resample('10L').ffill()
            resampled_data.append(data)
            data.index = pd.to_timedelta(data.index).astype(np.int64)/1E9

        return resampled_data  # return the resampled data

    # Filter the data
    def FilterData(self, data_series):
        filtered_data = []
        butterworth_order = 5
        critical_frequency = 30
        b, a = signal.butter(butterworth_order, critical_frequency,
                             btype='lowpass', analog=False, fs=100)

        for data in data_series:
            data.iloc[:] = signal.filtfilt(b, a, data.values.tolist(), axis=0)
            filtered_data.append(data)

        return filtered_data  # return the filtered data

    # Plot data and select analysis endpoints
    def CutData(self, data_series):
        cut_data = []
        fig, axes = plt.subplots(nrows=5, ncols=1)
        iter = 1
        for data in data_series:
            plt.subplot(5, 1, iter)
            plt.plot(data.index, data.values)
            plt.ylim(np.percentile(data.values, 0.1)-0.1,
                     np.percentile(data.values, 99)+0.1)
            iter = iter+1
        limits = plt.ginput(2)
        time_0 = round(limits[0][0], 1)
        time_f = round(limits[1][0], 1)
        print("Cutting data between ", time_0, " and ", time_f, " s")

        for data in data_series:
            data = data.loc[time_0:time_f]
            cut_data.append(data)

        plt.close()
        return cut_data  # return the cut data

    #
    def CalculateThrust(self):
        self.thrust = -1*self.imu['accelerometer_m_s2[2]'].values*self.mass
        self.thrust_cmd = self.control["control[3]"].values
        return