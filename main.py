# /usr/bin/python3
import os
import sys
from drone_thrust_identification import ThrustIdentification
from identification_flight_struct import PX4DroneIdentificationData

# main function


def main():
    data_path1 = "example_data/drone1-9/"
    data_path2 = "example_data/drone1-10/"
    data_path3 = "example_data/drone1-11/"
    data_path4 = "example_data/drone1-8/"

    # process flight data
    flight1 = PX4DroneIdentificationData(data_path1, mass=1.32)
    flight2 = PX4DroneIdentificationData(data_path2, mass=1.085)
    flight3 = PX4DroneIdentificationData(data_path3, mass=1.48)
    flight4 = PX4DroneIdentificationData(data_path4, mass=1.32)

    # do identification
    ID = ThrustIdentification(data=[flight1, flight2], validation=[flight3, flight4])


# define main
if __name__ == "__main__":
    print("--- START of identification_v2 script --- ")
    main()
    print("--- END of identification_v2 script --- ")
