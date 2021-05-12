import argparse
import signal
import time

import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal

from KMLMissionGeneration import KMLMissionGenerator
from SensorModule import WaterQualityModule


def obtener_ip_puerto(file_name='/etc/default/ardurover'):
    if DEBUG:
        return 'tcp:127.0.0.1:5762'
    with open(file_name, 'r') as f:
        # Tenemos que buscar la linea TELEM4=
        line = f.readline()
        while line.find("TELEM4") != 0:
            line = f.readline()
        datos = line.split('"')
        # este dato de abajo deberia ser algo como 'tcp:127.0.0.1:5760'
        return datos[1][3:]


keep_going = True
DEBUG = True

mg = KMLMissionGenerator('MisionesLoyola.kml')

waypoints = mg.get_mission_list()[0]


def manejador_de_senal(_, __):
    global keep_going
    keep_going = False


def arm(_vehicle):
    """
    Arms vehicle
    """
    # while not _vehicle.is_armable:
    #     print(" Waiting for vehicle to initialize...")
    #     time.sleep(1)

    # Copter should arm in GUIDED mode
    _vehicle.mode = VehicleMode("GUIDED")
    _vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not _vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)


def get_next_wp(_vehicle):
    global keep_going
    nextwp = waypoints.pop(0)
    print("Next waypoint is", nextwp)
    if len(waypoints) == 0:
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

    a = np.sin(0.5 * d_lat) ** 2 + np.sin(0.5 * d_lon) ** 2 * np.cos(lat1) * np.cos(lat2)
    c = 2.0 * np.arctan2(np.sqrt(a), np.sqrt(1.0 - a))
    return 6378100.0 * c < 0.5


if __name__ == '__main__':

    # Creamos el objeto del modulo del AutoPilot
    # vehicle = connect('tcp:127.0.0.1:5762', timeout=6, baud=115200, wait_ready=True)
    vehicle = connect(obtener_ip_puerto(), timeout=15)

    # Creamos el objeto de modulo de sensores #
    if not DEBUG:

        pump_parameters = {'charging_time': 7,
                           'discharging_time': 2,
                           'serial_string': '/dev/ttyACM0',
                           'mode': 'BuiltInPin'}

        modulo_de_sensores = WaterQualityModule(database_name='LOCAL_DATABASE.db',
                                                USB_string='USBPort1',
                                                timeout=6,
                                                baudrate=115200,
                                                pump_parameters=pump_parameters)
    arm(vehicle)
    print("Starting mission")

    # p misiones mas grandes keep_going debe bajar a false si vehicle.battery.level < 0.6

    while keep_going:

        if not vehicle.armed or vehicle.mode != VehicleMode("GUIDED"):
            break

        # # Mision Objetivo 0 : Ir a waypoint
        point2go = get_next_wp(vehicle)
        vehicle.simple_goto(point2go)

        while not reached_position(vehicle.location.global_relative_frame, point2go):
            time.sleep(1)
            continue

        # # Mision Objetivo 1 : Tomar muestra
        # Tomamos muestras continuamente #
        vehicle.mode = VehicleMode("LOITER")
        print("TOMANDO MUESTRAS", len(waypoints))

        if DEBUG:
            time.sleep(3)
        else:
            position = vehicle.location.global_relative_frame
            modulo_de_sensores.take_a_sample(position=[position.lat, position.lon], num_of_samples=1)

        # # Parte de Mision Objetivo 0: Ir a waypoint
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)

        # # Mision Objetivo 2:  Manual // Stand by
        # vehicle.mode = VehicleMode("MANUAL")
        # time.sleep(1)

    # Cerramos la conexion con el navio2
    print('FINISHING MISSION')
    vehicle.disarm()
    vehicle.close()

    # Cerramos la conexion con los sensores #
    modulo_de_sensores.close()
