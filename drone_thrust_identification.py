# /usr/bin/python3
import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt

from scipy import signal
from scipy import optimize

from identification_flight_struct import PX4DroneIdentificationData


# Normalized Thrust Identification
class ThrustIdentification():
    def __init__(self, data, validation=[]):

        for thing in data:
            if type(thing) is not PX4DroneIdentificationData:
                print(
                    " \n \n ERROR !!!All data must be PX4DroneIdentification objects \n \n")

        for thing in validation:
            if type(thing) is not PX4DroneIdentificationData:
                print(
                    " \n \n ERROR !!!All validation must be PX4DroneIdentification objects \n \n")

        self.data_set = data
        self.validation_set = validation

        # regression data
        self.voltage = np.array([])
        self.thrust = np.array([])
        self.thrust_cmd = np.array([])

        # validation data
        self.v_voltage = np.array([])
        self.v_thrust = np.array([])
        self.v_thrust_cmd = np.array([])

        # validation data
        print(" !--- Gathering Data from Experiments ---!")
        self.GatherData()

        self.best_fit_params = []

        print(" !--- Doing Identification ---!")
        self.DoRegression()

        print(" !--- Checking Results ---!")
        if validation is not []:
            self.CheckResults(validation)

    def GatherData(self):
        for data in self.data_set:
            self.thrust = np.append(self.thrust, data.thrust)
            self.voltage = np.append(self.voltage, data.voltage)
            self.thrust_cmd = np.append(self.thrust_cmd, data.thrust_cmd)

        for data in self.validation_set:
            self.v_thrust = np.append(self.v_thrust, data.thrust)
            self.v_voltage = np.append(self.v_voltage, data.voltage)
            self.v_thrust_cmd = np.append(self.v_thrust_cmd, data.thrust_cmd)

        return
    #

    def DoRegression(self):
        y_data = self.thrust
        x_data = np.stack((self.thrust_cmd, self.voltage), axis=0)
        print("x_data=", x_data)

        params, params_covariance = optimize.curve_fit(
            thrust_test_fnc, x_data, y_data)

        print("Params:", params)
        print("Params Covariance:  ", params_covariance)
        self.best_fit_params = params

        return

    def CheckResults(self, validation_data_path):
        if self.validation_set == []:
            print("There is not external cross-validation data")
            return

        else:
            measured_validation_thrust = self.v_thrust
            control_thrust = self.CallTestFunction(
                np.stack((self.v_thrust_cmd, self.v_voltage), axis=0))

            rms_error = np.sqrt(
                np.mean(np.square(measured_validation_thrust-control_thrust)))
            print("RMS VALIDATION ERROR: ", rms_error)

            plt.figure()
            plt.plot(measured_validation_thrust)
            plt.plot(control_thrust)
            plt.plot(np.abs(measured_validation_thrust - control_thrust))
            plt.ylim(0, np.percentile(measured_validation_thrust, 95))

            plt.show()
            pass

    def CallTestFunction(self, x):
        if len(self.best_fit_params) == 2:
            return thrust_test_fnc(x, self.best_fit_params[0], self.best_fit_params[1])
        elif len(self.best_fit_params) == 3:
            return thrust_test_fnc(x, self.best_fit_params[0], self.best_fit_params[1], self.best_fit_params[2])


def thrust_test_fnc(x, a, b, c):
    # print("HERE I AM")
    # print(x[0] ,x[1])
    # return a*x[0] + b*x[1]
    return a*x[0] + b*x[1] + c
    # return a*x + b

# def thrust_test_fnc(x, a, b, c):
#   return a*np.square(x) + b*x + c

# def thrust_test_fnc(x, a, b):
#   return a*(b*np.square(x) + (1-b)*x)
