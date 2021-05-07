
from SensorModule import WaterQualityModule
import signal
from time import sleep

keep_going = True

def manejador_de_senal(signum, frame):
    global keep_going
    # Si entramos en el manejador por una llamada CTRL-C, ponemos el flag a False
    keep_going = False

signal.signal(signal.SIGTERM,manejador_de_senal)


if __name__ == '__main__':

    # Creamos el objeto de modulo de sensores #

    pump_parameters = {'charging_time': 7,
                       'discharging_time': 2,
                       'serial_string': '/dev/ttyACM0',
                       'mode': 'BuiltInPin'}

    modulo_de_sensores = WaterQualityModule(database_name = 'LOCAL_DATABASE.db',
                                            USB_string = 'USBPort1',
                                            timeout = 6,
                                            baudrate = 115200,
                                            pump_parameters=pump_parameters)

    while keep_going:

        # Esperamos 2 segundos #
        sleep(2)
        # Tomamos muestras continuamente #
        modulo_de_sensores.take_a_sample(position = [0.0, 0.0],
                                         num_of_samples = 1)

    # Cerramos la conexion con los sensores #
    modulo_de_sensores.close()



