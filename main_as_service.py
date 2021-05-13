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
from SensorModule import WaterQualityModule


verbose = 1


def obtener_ip_puerto(file_name='/etc/default/ardurover'):
    """
    Reads from the file 'file_name', allegedly the Ardupilot config file, to find the TELEM4 connection string

    Args:
        file_name: Path of the Ardupilot TELEMETRY configuration file.

    Returns:
        The connection string for Dronekit to create a connection.
    """

    if DEBUG: # In debug mode, return the std connection string.
        return 'tcp:127.0.0.1:5762'

    with open(file_name, 'r') as f:
        # Look for TELEM4= line
        line = f.readline()
        while line.find("TELEM4") != 0 or len(line) == 0:
            line = f.readline()

        # If there is not defined, raise an error
        if len(line) == 0:
            raise EOFError("Reached EOF without obtaining TELEM4 in file.")
        datos = line.split('"')

        # Return the data. It must be similar to: 'tcp:127.0.0.1:5760'
        return datos[1][3:]


def manejador_de_senal(_, __):
    """
    An asyncronous signal handler to kill gracefully the process

    Args:
    _: Discard the input argument.
    __: Discard the input argument.

    """
    global keep_going
    keep_going = False


def asv_send_info():
    """
        A function that sends the ASV information to the coordinator in the MQTT Broker every 0.5 seconds.
    """

    while keep_going:
        msg = json.dumps({
            "Latitude": vehicle.location.global_relative_frame.lat,
            "Longitude": vehicle.location.global_relative_frame.lon,
            "yaw": vehicle.attitude.yaw,
            "veh_num": 1,
            "battery": vehicle.battery.level,
            "armed": vehicle.armed
        })  # Must be a JSON format file.
        mqtt.send_new_msg(msg)  # Send the MQTT message
        time.sleep(0.5)  # Sleep


def arm(_vehicle):
    """
    Arming vehicle function. To arm, the vehicle must be in GUIDED mode. If so, the _vehicle.armed flag
    can be activated. The function waits for the vehicle to be armed.

    Args:
         _vehicle: `dronekit.connection object.

    """
    # Copter should arm in GUIDED mode
    _vehicle.mode = VehicleMode("GUIDED")
    _vehicle.armed = True  # TODO: No es mas adecuado usar _vehicle.arm(). El while abajo no sirve (?)

    # Confirm vehicle armed before attempting to take off
    while not _vehicle.armed:
        if verbose > 0:
            print(" Waiting for arming...")
        time.sleep(1)


def get_next_wp(_vehicle):
    """
    Receives the vehicle object and pop the next waypoint. If `current_asv_mode is 1 (preloaded mission), pops the next
    mission waypoint. If `current_asv_mode is 3 (simple go-to), pop the position of the received mqtt wwaypoint.

    Args:
        _vehicle: The connection vehicle object from `dronekit.

    Returns:
        Returns the next waypoint as a `dronekit.LocationGlobal object.
    """

    global keep_going  # TODO: Es esto necesario?
    global received_mqtt_wp # TODO: Revisar esto. Creo que es necesario ponerlo como global. Ver más abajo.

    if current_asv_mode == 1:  # Preloaded
        nextwp = waypoints.pop(0)
    elif current_asv_mode == 3:
        nextwp = received_mqtt_wp # TODO: Atención. Esto esta en un scope superior. No deberíamos ponerlo como global (?)
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

    Args:
        current_loc: Actual position (dronekit.LocationGlobal).
        goal_loc: Reference position (dronekit.LocationGlobal).

    Returns:
        'True' if the ASV distance respecto to the target Waypoint is less than 1.5 meters.

    """

    # Convert to radians #
    lat1 = np.radians(current_loc.lat)
    lat2 = np.radians(goal_loc.lat)
    lon1 = np.radians(current_loc.lon)
    lon2 = np.radians(goal_loc.lon)

    # Obtains the latitude/longitude differences #
    d_lat = lat2 - lat1
    d_lon = lon2 - lon1

    # Returns True if the waypoint is within 1.5 meters the ASV position
    a = np.sin(0.5 * d_lat) ** 2 + np.sin(0.5 * d_lon) ** 2 * np.cos(lat1) * np.cos(lat2)
    c = 2.0 * np.arctan2(np.sqrt(a), np.sqrt(1.0 - a))
    return 6378100.0 * c < 1.5


def str2bool(v):
    """
    Simple conversion of STR argument to boolean value.

    Args:
        v: `str value

    Returns:
        `True/`False depending on the input parameter
    """

    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


def change_asv_current_mode(desired_mode):
    """
    Changes the ASV mode safelly

    Args:
        desired_mode: int` with the desired mode to change.

    Returns:
        Returns a boolean flag to indicate whether the mode has changed to the desired one or not.

    """

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
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py`

    Args:
        location1: Actual position (`dronekit.LocationGlobal`).
        location2: Reference position (`dronekit.LocationGlobal).

    Returns:
        The angle difference from `location1 to `location2


    """
    off_x = location2.lon - location1.lon
    off_y = location2.lat - location1.lat
    bearing = 90.00 + atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing


def move2wp():
    """
    Function for moving to the next wp. This function should only be called in mode 1 or 3 (Preloaded mission / simplegoto)
    because it uses `get_next_wp function.

    Returns:
        True when finished, False when it is not possible to move or change the mode.

    """
    global asv_mode

    # Arm the vehicle if needed #
    if not vehicle.armed:
        arm(vehicle)
    # If the vehicle cannot be armed or the autopilot mode is not GUIDED, raise a Warning and returns False.
    # The mode must be guided always to move to the next waypoint.
    if not vehicle.armed or vehicle.mode != VehicleMode("GUIDED"):
        if verbose > 0:
            print("Error: vehicle should be armed and in guided mode")
            print(f"but arming is {vehicle.armed} and in {vehicle.mode}.")
            print("Setting mode to Stand-by")
        asv_mode = 0
        return False

    # When the pre-requisites of armability and the correct mode are setted, obtain the next waypoint.
    # Depending on the mode, obtained from the preloaded mission or from the MQTT broker.
    point2go = get_next_wp(vehicle)

    # Throw some information if specified the verbose condition
    if verbose > 0:
        print("Turning to : ", get_bearing(vehicle.location.global_relative_frame, point2go), "N")
    condition_yaw(get_bearing(vehicle.location.global_relative_frame, point2go))
    time.sleep(2)

    # MOVE!
    vehicle.simple_goto(point2go)

    # Waits until the position has been reached.
    while not reached_position(vehicle.location.global_relative_frame, point2go):
        time.sleep(1)
        continue

    # Once the position has been reached, change the autopilot mode to LOITER to maintain actual position (disturbance
    # rejection)
    vehicle.mode = VehicleMode("LOITER")

    # Throw some information about the sampling
    if verbose > 0:
        if current_asv_mode == 1:
            print("TOMANDO MUESTRAS, quedan: ", len(waypoints), "waypoints")
        else:
            print("TOMANDO MUESTRAS")

    if DEBUG:
        time.sleep(3)
    else:
        # If not in Debugging, take a sample using the Sensor Module#
        position = vehicle.location.global_relative_frame
        modulo_de_sensores.take_a_sample(position=[position.lat, position.lon], num_of_samples=1)

    vehicle.mode = VehicleMode("GUIDED") # Return to GUIDED to pursue the next waypoint

    time.sleep(1) # Wait a second to be sure the vehicle mode is changed.

    return True


def on_message(_client, _, msg):
    """
    Asyncronous handler of a MQTT message. Ir receives a message from the broker. Depending on the fields of the input
    message, change the mode consequently.`

    Args:
        _client: Client object
        msg: MQTT message object.
    """

    global asv_mode, received_mqtt_wp
    if msg.topic == "veh1": # TODO: Este identificador tiene que cambiar para cada dron. Debería leerse de un archivo de configuracion unico para cada vehiculo.
        message = json.loads(msg.payload.decode('utf-8')) # Decode the msg into UTF-8

        if verbose > 0:
            print(f"Received {message} on topic {msg.topic}")
        if message["mission_type"] == "STANDBY": # Change the asv mission mode flag
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

    # para agilizar el inicio del script en modo DEBUG o RELEASE
    parser = argparse.ArgumentParser(description='Main ASV file.')
    parser.add_argument('-d', '--debug', default=True, type=str2bool,
                        help="Deactivates DEBUG mode if -d no, false, f, n or 0.")
    args = parser.parse_args()
    DEBUG = args.debug

    received_mqtt_wp = [0, 0, 0]  # Inicializando objeto de wp desde el servidor
    asv_mode = 0  # el modo del ASV deseado
    current_asv_mode = -1  # el modo del ASV actual

    asv_mode_strs = ["STANDBY", "GUIDED", "MANUAL", "SIMPLE", "RTL"]  # Strings para modos

    # TODO: Cuidado con esto. Debería leerse el addr de un archivo de configuracion del ASV por si cambia
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

    try:
        # Creamos el objeto del modulo del AutoPilot
        # vehicle = connect('tcp:127.0.0.1:5762', timeout=6, baud=115200, wait_ready=True)
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

        # creamos el hilo que continuamente envia datos de posicion al servidor
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
