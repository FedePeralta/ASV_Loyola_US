import os
from time import sleep

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
        os.system('echo "{}" > /sys/class/gpio/gpio{}/direction'.format(500 + channel - 1, direction))
        
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
        self.__clean_gpio(self.channel)
        print('GPIO object {self.channel} freed!')

class WaterPumpModule():

    def __init__(self,
                 activation_channel=8,
                 direction_channel=9,
                 reverse_direction=False,
                 charging_time=5,
                 discharging_time=5):

        assert activation_channel != direction_channel, "ACTIVATION CHN AND DIRECTION CHN MUST BE DIFFERENT!"
        assert charging_time > 0 and discharging_time > 0, "CHARGE/DISCHARGE TIME MUST BE GREATER THAN 0!"

        self.activation_pin = GPIO(channel=activation_channel, mode='out')
        self.direction_pin = GPIO(channel=direction_channel, mode='out')

        self.charging_time = charging_time
        self.discharging_time = discharging_time

        self.activation_pin.write_gpio('LOW')
        self.direction_pin = GPIO(channel=direction_channel, mode='out')
        self.reverse_direction = reverse_direction

    def charge_probe(self):

        # Set direction of water movement #
        if self.reverse_direction:
            self.direction_pin.write_gpio('HIGH')
        else:
            self.direction_pin.write_gpio('LOW')

        # Move water for a time #
        self.activation_pin.write_gpio('HIGH')
        sleep(self.charging_time)

        # Stop water #
        self.activation_pin.write_gpio('LOW')

    def discharge_probe(self):

        # Set direction of water movement #
        if self.reverse_direction:
            self.direction_pin.write_gpio('LOW')
        else:
            self.direction_pin.write_gpio('HIGH')

        # Move water for a time #
        self.activation_pin.write_gpio('HIGH')
        sleep(self.discharging_time)

        # Stop water #
        self.activation_pin.write_gpio('LOW')












