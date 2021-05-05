import os
from time import sleep
from ArduinoSerialBoard import SerialBoard

class GPIO():

    def __init__(self, channel = 8, mode = 'out'):

        """ Channek of PWM"""

        assert 14 >= channel >= 0, "OUTPUT CHANNEL NOT VALID. SELECT FROM 0 TO 14"
        assert mode in ['out', 'in'], "INVALID MODE! Select output/input"

        self.channel = channel
        self.mode = mode

        # Unexport GPIO utility #
        self.__clean_gpio(self.channel)
        # Export the GPIO utility
        self.__attach_gpio(self.channel)
        # Set direction #
        self.__set_gpio_mode(self.channel, self.mode)

    @staticmethod
    def __clean_gpio(channel):
        os.system('echo "{}" > /sys/class/gpio/unexport'.format(500 + channel - 1))

    @staticmethod
    def __set_gpio_mode(channel, direction):
        os.system('echo "{}" > /sys/class/gpio/gpio{}/direction'.format(direction,500 + channel - 1))
        
    @staticmethod
    def __attach_gpio(channel):
        os.system('echo "{}" > /sys/class/gpio/export'.format(500 + channel - 1))

    def read_gpio(self):
        value = os.system('cat /sys/class/gpio/gpio{}/value'.format(500 + self.channel - 1))
        return int(value)

    def write_gpio(self, value):
        assert value in ['HIGH', 'LOW'], "BAD WRITING VALUE! Select with HIGH/LOW"
        assert self.mode == 'out', "GPIO {self.channel} is not an OUTPUT pin!"
        wr_value = 0 if value == 'LOW' else 1
        os.system('echo "{}" > /sys/class/gpio/gpio{}/value'.format(wr_value, 500 + self.channel - 1))

    def __del__(self):
        self.write_gpio('LOW')
        self.__clean_gpio(self.channel)
        print('GPIO object {} freed!'.format(self.channel))

class WaterPumpModule():

    def __init__(self, serial_string="/dev/ttyUSB1", activation_channel=8, charging_time=5, discharging_time=5, mode = 'SerialBoard'):

        assert charging_time > 0 and discharging_time > 0, "CHARGE/DISCHARGE TIME MUST BE GREATER THAN 0!"
        assert mode in ['SerialBoard', 'BuiltInPin'], "SerialBoard/BuiltInPin are the only permitted modes!"

        if mode == 'BuiltInPin':
            self.activation_pin = GPIO(channel=activation_channel, mode='out')
        else:
            self.activation_pin = SerialBoard(ser_path=serial_string, baudrate=9600, timeout=None)

        self.charging_time = charging_time
        self.discharging_time = discharging_time

        self.activation_pin.write_gpio('LOW')

    def charge_probe(self):

        # Move water for a time #
        self.activation_pin.write_gpio('HIGH')
        # Wait until the probe is full #
        sleep(self.charging_time)
        # When full, deactivate the pump #
        self.activation_pin.write_gpio('LOW')

    def discharge_probe(self):

        # Activate the pump to discharge
        self.activation_pin.write_gpio('HIGH')
        # Wait for the discharge time
        sleep(self.discharging_time)
        # Deactivate the pump
        self.activation_pin.write_gpio('LOW')















