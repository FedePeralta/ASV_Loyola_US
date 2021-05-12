import argparse
import json
import signal
import threading
import time
from math import atan2

import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal
from pymavlink import mavutil

from KMLMissionGeneration import KMLMissionGenerator
from MQTT import MQTT

# from SensorModule import WaterQualityModule

verbose = 1


def obtener_ip_puerto(file_name='/etc/default/ardurover'):
    if DEBUG:
        return 'tcp:127.0.0.1:5762'
    with open(file_name, 'r') as f:
        # Tenemos que buscar la linea TELEM4=
        line = f.readline()
        while line.find("TELEM4") != 0 or len(line) == 0:
            line = f.readline()

        if len(line) == 0:
            raise EOFError("Reached EOF without obtaining TELEM4 in file.")
        datos = line.split('"')
        # este dato de abajo deberia ser algo como 'tcp:127.0.0.1:5760'
        return datos[1][3:]


def manejador_de_senal(_, __):
    global keep_going
    keep_going = False


def asv_send_info():
    while keep_going:
        msg = json.dumps({
            "Latitude": vehicle.location.global_relative_frame.lat,
            "Longitude": vehicle.location.global_relative_frame.lon,
            "yaw": vehicle.attitude.yaw,
            "veh_num": 1,
            "battery": vehicle.battery.level,
            "armed": vehicle.armed
        })
        mqtt.send_new_msg(msg)
        time.sleep(0.5)


def arm(_vehicle):
    """
    Arms vehicle
    """

    # Copter should arm in GUIDED mode
    _vehicle.mode = VehicleMode("GUIDED")
    _vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not _vehicle.armed:
        if verbose > 0:
            print(" Waiting for arming...")
        time.sleep(1)


def get_next_wp(_vehicle):
    global keep_going
    if current_asv_mode == 1:  # Preloaded
        nextwp = waypoints.pop(0)
    elif current_asv_mode == 3:
        nextwp = received_mqtt_wp
    else:
        raise ValueError(f"Current ASV Mode should be 1: {asv_mode_strs[1]} or 3: {asv_mode_strs[3]}.")
    if verbose > 0:
        print("Next waypoint is", nextwp)
    return LocationGlobal(nextwp[1], nextwp[0], nextwp[2])


def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting
    the yaw using this function there is no way to return to the default yaw "follow direction
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see:
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


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
    return 6378100.0 * c < 1.5


def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


def change_asv_current_mode(desired_mode):
    global current_asv_mode
    if current_asv_mode != desired_mode:
        current_asv_mode = desired_mode
        if verbose > 0:
            print(f"Changed current ASV mode to {asv_mode_strs[current_asv_mode]}.")
        return True
    return False


def get_bearing(location1, location2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    off_x = location2.lon - location1.lon
    off_y = location2.lat - location1.lat
    bearing = 90.00 + atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing


def move2wp():
    if not vehicle.armed:
        arm(vehicle)
    if not vehicle.armed or vehicle.mode != VehicleMode("GUIDED"):
        global asv_mode
        if verbose > 0:
            print("Error: vehicle should be armed and in guided mode")
            print(f"but arming is {vehicle.armed} and in {vehicle.mode}.")
            print("Setting mode to Stand-by")
        asv_mode = 0
        return False

    point2go = get_next_wp(vehicle)

    if verbose > 0:
        print("Turning to : ", get_bearing(vehicle.location.global_relative_frame, point2go), "N")
    condition_yaw(get_bearing(vehicle.location.global_relative_frame, point2go))
    time.sleep(2)

    vehicle.simple_goto(point2go)

    while not reached_position(vehicle.location.global_relative_frame, point2go):
        time.sleep(1)
        continue

    vehicle.mode = VehicleMode("LOITER")
    if verbose > 0:
        print("TOMANDO MUESTRAS, quedan: ", len(waypoints), "waypoints")

    if DEBUG:
        time.sleep(3)
    else:
        position = vehicle.location.global_relative_frame
        modulo_de_sensores.take_a_sample(position=[position.lat, position.lon], num_of_samples=1)
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(1)
    return True


def on_message(_client, _, msg):
    global asv_mode, received_mqtt_wp
    if msg.topic == "veh1":
        message = json.loads(msg.payload.decode('utf-8'))
        if verbose > 0:
            print(f"Received {message} on topic {msg.topic}")
        if message["mission_type"] == "STANDBY":
            asv_mode = 0
        elif message["mission_type"] == "GUIDED":
            asv_mode = 1
        elif message["mission_type"] == "MANUAL":
            asv_mode = 2
        elif message["mission_type"] == "SIMPLE":
            asv_mode = 3
            received_mqtt_wp = [message["lon"], message["lat"], 0]
        elif message["mission_type"] == "RTL":
            asv_mode = 4


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Main ASV file.')
    parser.add_argument('-d', '--debug', default=True, type=str2bool,
                        help="Deactivates DEBUG mode if -d no, false, f, n or 0.")
    args = parser.parse_args()

    DEBUG = args.debug
    received_mqtt_wp = [0, 0, 0]
    asv_mode = 0  # el modo deseado
    current_asv_mode = -1  # el modo actual
    asv_mode_strs = ["STANDBY", "GUIDED", "MANUAL", "SIMPLE", "RTL"]

    mqtt = MQTT("1", addr='52.232.74.235', topics2suscribe=["veh1"], on_message=on_message)

    vehicle_ip = obtener_ip_puerto()

    if verbose > 0:
        print("ASV ready to connect.")
        print(f"Starting mode:{asv_mode} {asv_mode_strs[asv_mode]}.")
        print(f"Debug set to {DEBUG}.")
        print(f"Connecting to vehicle in {vehicle_ip}")

    keep_going = True
    signal.signal(signal.SIGTERM, manejador_de_senal)

    mg = KMLMissionGenerator('MisionesLoyola.kml')
    waypoints = mg.get_mission_list()[0]

    # Creamos el objeto del modulo del AutoPilot
    # vehicle = connect('tcp:127.0.0.1:5762', timeout=6, baud=115200, wait_ready=True)
    try:
        vehicle = connect(vehicle_ip, timeout=15)
        # Creamos el objeto de modulo de sensores #
        if DEBUG:
            vehicle.groundspeed = 1.0
            modulo_de_sensores = False
        else:
            pump_parameters = {'charging_time': 7,
                               'discharging_time': 2,
                               'serial_string': '/dev/ttyACM0',
                               'mode': 'BuiltInPin'}

            modulo_de_sensores = WaterQualityModule(database_name='LOCAL_DATABASE.db',
                                                    USB_string='USBPort1',
                                                    timeout=6,
                                                    baudrate=115200,
                                                    pump_parameters=pump_parameters)
        threading.Thread(target=asv_send_info, ).start()
    except ConnectionRefusedError:
        keep_going = False
        if verbose > 0:
            print("Connection to navio2 could not be made")

    # p misiones mas grandes keep_going debe bajar a false si vehicle.battery.level < 0.6
    while keep_going:
        if asv_mode == 0:  # Stand By
            if change_asv_current_mode(asv_mode):
                if vehicle.armed:
                    vehicle.disarm()
                if verbose > 0:
                    print("ASV is armed." if vehicle.armed else "ASV is disarmed. Standing By.")
            time.sleep(1)
        elif asv_mode == 1:  # Pre-loaded Mission
            if verbose > 0 and change_asv_current_mode(asv_mode):
                print("Starting Pre-loaded Mission.")

            if len(waypoints) == 0:
                if verbose > 0:
                    print(f"Finished preloaded mission.")
                    print("Setting mode to Stand-by.")
                asv_mode = 0
                continue

            if move2wp():
                time.sleep(1)
        elif asv_mode == 2:  # Manual Mode
            if change_asv_current_mode(asv_mode):
                if vehicle.mode != VehicleMode("MANUAL"):
                    vehicle.mode = VehicleMode("MANUAL")
                if verbose > 0:
                    print(f"Vehicle is now in {vehicle.mode}")
            else:
                time.sleep(1)
        elif asv_mode == 3:  # Simple Go-To
            if change_asv_current_mode(asv_mode):
                if move2wp():
                    if verbose > 0:
                        print("Finished simple goto.")
                        print("Setting mode to Stand-by.")
                    asv_mode = 0
        elif asv_mode == 4:  # RTL
            if change_asv_current_mode(asv_mode):
                if vehicle.mode != VehicleMode("RTL"):
                    vehicle.mode = VehicleMode("RTL")
                if verbose > 0:
                    print(f"Vehicle is now in {vehicle.mode}")
            else:
                time.sleep(1)
    # Cerramos la conexion con el navio2
    mqtt.client.disconnect()
    if verbose > 0:
        print('FINISHING MISSION')
    try:
        vehicle.disarm()
        vehicle.close()
    except NameError:
        pass

    # Cerramos la conexion con los sensores #
    if not DEBUG:
        modulo_de_sensores.close()
