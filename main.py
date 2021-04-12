import signal
import time

import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal

from KMLMissionGeneration import KMLMissionGenerator

keep_going = True

mg = KMLMissionGenerator('kml_files/Misiones_Dron_US_Loyola_2.kml')

loyola_wps = mg.get_mission_list()[0]


def manejador_de_senal(_, __):
    global keep_going
    keep_going = False


def arm(_vehicle):
    """
    Arms vehicle
    """
    while not _vehicle.is_armable:
        print(" Waiting for vehicle to initialize...")
        time.sleep(1)

    # Copter should arm in GUIDED mode
    _vehicle.mode = VehicleMode("GUIDED")
    _vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not _vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)


def get_next_loyola_lake_wp(_vehicle):
    global keep_going
    nextwp = loyola_wps.pop(0)
    print("Next waypoint is", nextwp)
    if len(loyola_wps) == 0:
        keep_going = False
    return LocationGlobal(nextwp[1], nextwp[0], nextwp[2])


signal.signal(signal.SIGTERM, manejador_de_senal)


def reached_position(current_loc, goal_loc):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """

    lat1 = np.radians(current_loc.lat)
    lat2 = np.radians(goal_loc.lat)
    lon1 = np.radians(current_loc.lon)
    lon2 = np.radians(goal_loc.lon)
    d_lat = lat2 - lat1
    d_lon = lon2 - lon1

    a = np.sin(0.5 * d_lat)**2 + np.sin(0.5 * d_lon)**2 * np.cos(lat1) * np.cos(lat2)
    c = 2.0 * np.arctan2(np.sqrt(a), np.sqrt(1.0 - a))
    return 6378100.0 * c < 2.5

    # dlat = goal_loc.lat - current_loc.lat
    # dlong = goal_loc.lon - current_loc.lon
    # print(dlat, dlong, np.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5)
    # return np.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5 < 2.5


if __name__ == '__main__':

    # Creamos el objeto del modulo del AutoPilot
    vehicle = connect('tcp:127.0.0.1:5760', timeout=6, baud=115200, wait_ready=True)
    # vehicle = connect('dev/ttyUSBO', timeout=6, baud=115200)
    # Creamos el objeto de modulo de sensores #
    # modulo_de_sensores = WaterQualityModule(database_name='LOCAL_DATABASE.db',
    #                                         USB_string='USB1',
    #                                         timeout=6,
    #                                         baudrate=115200)
    arm(vehicle)
    print("Starting mission")

    while keep_going:
        point2go = get_next_loyola_lake_wp(vehicle)
        vehicle.simple_goto(point2go)
        while not reached_position(vehicle.location.global_relative_frame, point2go):
            continue
        # Tomamos muestras continuamente #
        vehicle.mode = VehicleMode("LOITER")
        print("TOMANDO MUESTRAS", len(loyola_wps))
        time.sleep(3)
        # modulo_de_sensores.take_a_sample(position=[0.0, 0.0],
        #                                  num_of_samples=1)
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)

    # Cerramos la conexion con el navio2
    vehicle.close()

    # Cerramos la conexion con los sensores #
    # modulo_de_sensores.close()
