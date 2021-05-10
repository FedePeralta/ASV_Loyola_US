import signal
from PumpModule import WaterPumpModule
keep_going = True
import time

def manejador_de_senal(signum, frame):
    global keep_going
    # Si entramos en el manejador por una llamada CTRL-C, ponemos el flag a False
    keep_going = False

signal.signal(signal.SIGTERM,manejador_de_senal)


if __name__ == '__main__':

    # Creamos el objeto de modulo de sensores #
    bomba = WaterPumpModule(serial_string = "/dev/ttyACM0",
                            charging_time=5,
                            discharging_time=5,
                            mode = 'BuiltInPin')

    while keep_going:

        # Esperamos 1 segundo
        time.sleep(1)
        # Cargamos la bomba
        bomba.charge_probe()
        # Esperamos un segundo después de cargarla
        time.sleep(1)
        # La descargamos
        bomba.discharge_probe()


    # Eliminamos el objeto bomba #
    del bomba