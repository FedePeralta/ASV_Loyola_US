import serial
import time
import sqlite3
from datetime import datetime
from .PumpModule import WaterPumpModule


class WaterQualityModule():

    """ Class for the water quality module sensor aka Libellium. """

    def __init__(self, database_name = 'LOCAL_DATABASE.db', USB_string = 'USBPort1', timeout = 6, baudrate = 115200, pump_parameters = None):

        """ Create the serial object """
        self.serial = serial.Serial('/dev/tty' + USB_string, baudrate, timeout=timeout)

        """ Connect to the Database """
        self.database = sqlite3.connect(database_name)

        """ Create the cursor to manage the database"""
        self.cursor = self.database.cursor()

        """ Initialize the data dictionary of the sensor measurements """
        self.sensor_data = {}

        """ Create the Pump Module object """
        if pump_parameters is None:
            pump_parameters = {'charging_time': 6,
                               'discharging_time': 1.2,
                               'serial_string': '/dev/ttyACM0',
                               'mode': 'BuiltInPin'}

        self.pump = WaterPumpModule(charging_time=pump_parameters['charging_time'],
                                    discharging_time=pump_parameters['discharging_time'],
                                    mode=pump_parameters['mode'],
                                    serial_string=pump_parameters['serial_string'])


    def take_a_sample(self, position, num_of_samples = 1):
        """ Take num_of_samples and save the data with the given position in the database """

        """ Charge the pump!"""
        self.pump.charge_probe()

        # Iterate over the sample_nums
        for _ in range(num_of_samples):

            sample_adquisition_status = False  # Mientras no se tome una muestra correcta, iteramos

            while sample_adquisition_status is False:

                time.sleep(0.5)  # Polling time. Every 0.5 secs, check the buffer #

                # Leemos todo el puerto serie #
                line = self.serial.read(self.serial.in_waiting)

                if line == b'':  # He leido un buffer vacio
                    continue
                else:
                    tramas = str(line)  # Pasamos a string
                    tramas = tramas.split('<=>')  # Dividimos entre tramas

                    if len(tramas) < 2 or len(tramas) > 2:  # El mensaje la primera vez no se recibe bien
                        continue

                    trama = tramas[1]  # Cogemos la primera trama del buffer

                    # Dividimos entre campos
                    # Formato de paquete ION: #
                    # line_str = [JUNK, JUNK, ID, SAMPLE_NUM, BATM, TEMP, PH, DO, COND, ORP, eop]
                    # Formato de paquete no-ION: #
                    # line_str = [JUNK, JUNK, ID, SAMPLE_NUM, BATM, TEMP, PH, COND, ORP, eop]

                    line_str = trama.split('#')

                    if len(line_str) > 11 or len(line_str) < 10:  # algunas veces el mensaje no se recibe entero
                        print('ERROR DE FORMATO')
                        continue

                    # Descartamos los dos primeros campos y el ultimo (JUNK)
                    line_str = line_str[2:-1]

                    # La línea de código anterior separa los caracteres del principio de la lectura que "no tienen"
                    # importancia en la lectura de los sensores
                    # Se han identificado y separado las partes que interesan de la cadena,
                    # es decir, se intenta eliminar
                    # o separar los caracteres que no se interpretan como parte de los sensore

                    # Iteramos para cada uno de los valores en la cadena #
                    self.sensor_data['DO'] = -1.0  # Como no es seguro que el SW tenga este sensor,
                    # le damos un valor -1 predeterminado.

                    for indx, campo in enumerate(line_str):

                        if indx == 0:  # El primer campo no es un sensor, es el nombre del SW
                            self.sensor_data['ID'] = campo
                        elif indx == 1:  # El segundo campo tampoco es un sensor, es el numero de muestra #
                            self.sensor_data['SAMPLE_NUM'] = campo
                        else:  # El resto si son sensores
                            sensor, valor = campo.split(':')  # Cada campo de line_str tiene como formato "SENSOR:VALOR"
                            self.sensor_data[sensor] = float(valor)  # Almacenamos el valor leido en el
                            # campo que indica el nombre sensor

                    str_date = str(datetime.now())  # Leemos la fecha y la convertimos en string
                    # Metemos la fecha en el diccionario de variables
                    self.sensor_data['DATE'] = str_date

                    # Almacenamos la posicion
                    self.sensor_data['LATITUD'] = position[0]
                    self.sensor_data['LONGITUD'] = position[1]

                    print(self.sensor_data)

                    # creamos una tupla de parámetros que nos permitirá introducir los datos en la tabla sensor
                    parametros = (self.sensor_data['ID'],
                                  self.sensor_data['SAMPLE_NUM'],
                                  self.sensor_data['BAT'],
                                  self.sensor_data['WT'],
                                  self.sensor_data['PH'],
                                  self.sensor_data['DO'],
                                  self.sensor_data['LATITUD'],
                                  self.sensor_data['LONGITUD'],
                                  self.sensor_data['COND'],
                                  self.sensor_data['ORP'],
                                  self.sensor_data['DATE'])

                    # insertamos valores en nuestra tabla "sensor"
                    self.cursor.execute(
                        "INSERT INTO sensor (ID,SAMPLE_NUM,BAT,TEMP,PH,DO,LATITUD,LONGITUD,COND,ORP,DATE) VALUES(?,?,?,?,?,?,?,?,?,?,?)",
                        parametros)

                    # el siguiente comando "commit()" guarda la tabla creada
                    self.database.commit()

                    # Marcamos el flag de muestra guardada con exito
                    sample_adquisition_status = True

        """ Discharge the pump!"""
        self.pump.discharge_probe()

        return True

    def close(self):

        print("Cerrando base de datos!")
        self.database.close()  # Cerramos la DB
        print("Base de datos cerrada")
        print("Cerrando puerto serie!")
        self.serial.close()  # Cerramos la com. serie
        print("Puerto serie cerrado!")

    def __del__(self):

        print("Cerrando base de datos!")
        self.database.close()  # Cerramos la DB
        print("Base de datos cerrada")
        print("Cerrando puerto serie!")
        self.serial.close()  # Cerramos la com. serie
        print("Puerto serie cerrado!")

