import signal
import time

import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative


def obtener_ip_puerto(file_name='/etc/default/ardurover'):
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


def manejador_de_senal(_, __):
    global keep_going
    keep_going = False


def arm(_vehicle):
    """
    Arms vehicle
    """

    # Copter should arm in GUIDED mode
    _vehicle.mode = VehicleMode("GUIDED")
    _vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not _vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)


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

    vehicle = connect(obtener_ip_puerto(), timeout=15)

    # Test 0:
    print("Test preparado. Para iniciar arme y desarme el vehiculo con el mando")
    while not vehicle.armed:
        time.sleep(0.2)
        pass
    print("1/3: El vehiculo no está armado")
    while vehicle.armed:
        pass
    print("2/3: El vehiculo está armado")
    while not vehicle.armed:
        pass
    print("3/3 Vehicle no está armado (Inicializando test): ", vehicle.armed)

    # Test 1 :
    print("Vehicle GPS Location : lat = ", vehicle.location.global_relative_frame.lat, ", lon = ",
          vehicle.location.global_relative_frame.lon)
    close_loc = LocationGlobalRelative(37.30744958158522, -5.940196719327942)
    print("Debe estar cerca de: ", close_loc, ". Menos de 0.5 m (true) = ",
          reached_position(vehicle.location, close_loc))

    # Test 2:
    print("Vehicle yaw: (Si apunta al N debe ser ~0) : ", vehicle.attitude.yaw)

    # Test 3:
    for i in range(5):
        print("Vehicle battery: Level = ", vehicle.battery.level)
        print("               : Voltage = ", vehicle.battery.voltage)
        print("               : Current = ", vehicle.battery.current)

    # Cerramos la conexion con el navio2
    print('FINISHING TEST')
    vehicle.disarm()
    vehicle.close()
