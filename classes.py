import RPi.GPIO as GPIO
import time
from varname import nameof
import arduino_methods

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
# The physical pin number is the key, the gpio number is the value
pin_to_gpio = {3: 2, 5: 3, 7: 4, 8: 14, 10: 15, 11: 17, 12: 18, 13: 27, 15: 22, 16: 23, 18: 24, 19: 10, 21: 9, 22: 25,
               23: 11, 24: 8, 26: 7, 27: 0, 28: 1, 29: 5, 31: 6, 32: 12, 33: 13, 35: 19, 36: 16, 37: 26, 38: 20, 40: 21}
arduinos, header = arduino_methods.generate_arduinos()


class Motor:
    def __init__(self, motor_name, direction_pin, step_pin, limit_switches, direction, step_resolution):
        self.directions = {'open': 1, 'close': 0}
        self.velocity = 0
        self.current_position = 0
        self.step_counter = 0
        self.stop = 0  # a condition triggered by the limit switch

        self.motor_name = motor_name
        self.direction_pin = direction_pin
        self.step_pin = step_pin
        self.step_resolution = step_resolution
        self.direction = self.directions[direction]

        self.limit_switches = limit_switches

        self.check_limit_switch()
        self.motor_setup()

    def motor_setup(self):
        # This function sets up the motor's pins for initial use
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)

    def check_limit_switch(self):
        # This function reads the off switch status and updates it's state in the object
        limit_switches_state = []

        if any(self.limit_switches.get_values()):
            self.stop = 1

        else:
            self.stop = 0

    def motor_single_step(self, velocity):
        # This function makes the motor moves a single step in a defined time interval
        GPIO.output(self.step_pin, GPIO.HIGH)
        time.sleep(0.5 * (1 / velocity))
        GPIO.output(self.step_pin, GPIO.LOW)
        time.sleep(0.5 * (1 / velocity))

    def update_motor_status(self, velocity, direction):
        # This function updates the motor's dynamic values
        self.step_counter += 1
        self.velocity = velocity
        self.direction = direction

        if self.direction == 1:
            self.current_position += self.step_resolution
        else:
            self.current_position -= self.step_resolution

        self.current_position = round(self.current_position, 3)

    def motor_full_single_step_update(self, velocity, direction):
        self.motor_single_step(velocity)
        self.update_motor_status(velocity, direction)
        self.check_limit_switch()

    def motor_while_loop(self, velocity, direction, next_position):
        # Main motor function, moves the motor and updates it's dynamic values
        self.direction = self.directions[direction]
        GPIO.output(self.direction_pin, self.direction)
        while not self.stop and velocity != 0 and self.current_position != next_position:
            self.motor_full_single_step_update(velocity, self.direction)

    def motor_for_loop(self, velocity, direction, step_amount):
        # Main motor function, moves the motor and updates it's dynamic values
        self.direction = self.directions[direction]
        GPIO.output(self.direction_pin, self.direction)
        for _step in range(step_amount):
            if not self.stop and velocity != 0:
                self.motor_full_single_step_update(velocity, self.direction)
            else:
                break

    def get_values(self):
        # This function returns all the motor's attributes
        return {'motor_name': self.motor_name,
                'step_resolution': self.step_resolution,
                'direction_pin': self.direction_pin,
                'step_pin': self.step_pin,
                'step_counter': self.step_counter,
                'direction': self.direction,
                'velocity': self.velocity,
                'current_position': self.current_position}

    def go_to_home_position(self, home_position):
        direction = self.direction
        velocity = 1000
        # Main motor function, moves the motor and updates it's dynamic values
        GPIO.output(self.direction_pin, direction)

        print(f'{self.motor_name}: going to position 0.')
        while not self.stop:
            self.motor_full_single_step_update(velocity, direction)

        self.current_position = 0
        print(f'{self.motor_name}: going to home position.')
        direction = not self.direction
        GPIO.output(self.direction_pin, direction)
        for _ in range(int(home_position / self.step_resolution)):
            self.motor_full_single_step_update(velocity, direction)


class AnalogSensor:
    def __init__(self, sensor_type, arduino_address, transform_function):
        self.sensor_type = sensor_type
        self.arduino_address = arduino_address
        self.transform_function = transform_function

    def transform(self, analog_data):
        #return self.transform_function(analog_data)
        return analog_data

    def read_sensor(self, analog_data):
        return self.transform(analog_data[self.arduino_address['pin']])


class DigitalSensor:
    def __init__(self, sensor_pin):
        self.sensor_pin = sensor_pin
        self.sensor_setup()
        self.state = self.check_state()

    def sensor_setup(self):
        GPIO.setup(pin_to_gpio[self.sensor_pin], GPIO.IN)

    def check_state(self):
        self.state = GPIO.input(pin_to_gpio[self.sensor_pin])
        return self.state


class AnalogSensorsGroup:
    def __init__(self, sensor_type, transform_function, **kwargs):
        self.transform_function = transform_function
        self.number_of_sensors = len(kwargs.keys())
        self.sensor_names = list(kwargs.keys())
        self.sensor_type = sensor_type
        for attr in kwargs.keys():
            self.__dict__[attr] = AnalogSensor(sensor_type=self.sensor_type, arduino_address=kwargs[attr],
                                               transform_function=self.transform_function)

    def get_values(self, arduino_data):
        states = {}
        for sensor_name in self.sensor_names:
            states[sensor_name] = self.__dict__[sensor_name].read_sensor(arduino_data)
        return states


class DigitalSensorsGroup:
    def __init__(self, sensor_type, **kwargs):
        self.number_of_sensors = len(kwargs.keys())
        self.sensor_names = list(kwargs.keys())
        self.sensor_type = sensor_type
        for attr in kwargs.keys():
            self.__dict__[attr] = DigitalSensor(sensor_pin=kwargs[attr])

    def get_values(self):
        states = {}
        for sensor_name in self.sensor_names:
            states[sensor_name] = self.__dict__[sensor_name].check_state()
        return states


class Arm:
    def __init__(self, limit_switches_kwargs, motor_name, motor_direction_pin,
                 motor_step_pin, motor_direction, motor_step_resolution, strain_gages_kwargs,
                 strain_gage_transform_function, thermistors_kwargs,
                 thermistor_transform_function, hdrm_pin_kwargs, arduino):
        self.limit_switches = DigitalSensorsGroup(sensor_type='limit switch', **limit_switches_kwargs)
        self.motor = Motor(motor_name=motor_name, direction_pin=motor_direction_pin, step_pin=motor_step_pin,
                           limit_switches=self.limit_switches, direction=motor_direction,
                           step_resolution=motor_step_resolution)
        self.strain_gages = AnalogSensorsGroup(sensor_type='strain_gages',
                                               transform_function=strain_gage_transform_function,
                                               **strain_gages_kwargs)
        self.thermistors = AnalogSensorsGroup(sensor_type='thermistors',
                                              transform_function=thermistor_transform_function,
                                              **thermistors_kwargs)
        self.hdrm = DigitalSensorsGroup(sensor_type='hdrm', **hdrm_pin_kwargs)
        self.arduino = arduino

    def get_arm_state(self):
        arduino_data = self.get_analog_data()
        return {'limit_switches': self.limit_switches.get_values(),
                'motor': self.motor.get_values(),
                'strain_gages': self.strain_gages.get_values(arduino_data),
                'thermistors': self.thermistors.get_values(arduino_data),
                'hdrm': self.hdrm.get_values()}

    def get_analog_data(self):
        return arduino_methods.get_all_analog_values(arduinos[f'arduino {self.arduino}'], header)


class EGSE:
    def __init__(self, arms_meta_data):
        self.arms_meta_data = arms_meta_data
        self.digital_pins = {3: 'free', 5: 'free', 7: 'free', 8: 'free', 10: 'free', 11: 'free', 12: 'free',
                             13: 'free', 15: 'free', 16: 'free', 18: 'free', 19: 'free', 21: 'free', 22: 'free',
                             23: 'free', 24: 'free', 26: 'free', 27: 'free', 28: 'free', 29: 'free', 31: 'free',
                             32: 'free', 33: 'free', 35: 'free', 36: 'free', 37: 'free', 38: 'free', 40: 'free'}
        self.analog_pins = {'arduino_0': {0: 'free', 1: 'free', 2: 'free', 3: 'free', 4: 'free', 5: 'free', 6: 'free',
                                          7: 'free', 8: 'free', 9: 'free', 10: 'free', 11: 'free', 12: 'free',
                                          13: 'free', 14: 'free', 15: 'free'},
                            'arduino_1': {0: 'free', 1: 'free', 2: 'free', 3: 'free', 4: 'free', 5: 'free', 6: 'free',
                                          7: 'free', 8: 'free', 9: 'free', 10: 'free', 11: 'free', 12: 'free',
                                          13: 'free', 14: 'free', 15: 'free'}
                            }
        self.generate_arms()

    def generate_arms(self):
        for i, attr in enumerate(self.arms_meta_data.keys()):
            arms_meta_data = self.arms_meta_data[attr]
            limit_switch_pins = self.assign_pins(arms_meta_data['limit_switches'], 'digital', attr)
            hdrm_pins = self.assign_pins(arms_meta_data['hdrms'], 'digital', attr)
            motor_pins = self.assign_pins(arms_meta_data['motor'], 'digital', attr)
            strain_gage_pins = self.assign_pins(arms_meta_data['strain_gages'], 'analog', attr)
            thermistor_pins = self.assign_pins(arms_meta_data['thermistors'], 'analog', attr)
            print(f'Generating {attr}')
            self.__dict__[attr] = Arm(
                arduino=arms_meta_data['arduino'],
                limit_switches_kwargs=limit_switch_pins,
                motor_name=f'motor #{i}',
                motor_direction_pin=motor_pins['motor_direction'],
                motor_step_pin=motor_pins['motor_step'],
                motor_direction='open',
                motor_step_resolution=0.05,
                strain_gages_kwargs=strain_gage_pins,
                strain_gage_transform_function=0,
                thermistors_kwargs=thermistor_pins,
                thermistor_transform_function=0,
                hdrm_pin_kwargs=hdrm_pins)

    def assign_pins(self, clients, pin_type, arm_name):
        for client in clients:
            if pin_type == 'analog':
                if self.analog_pins[f'arduino_{clients[client]["arduino"]}'][clients[client]["pin"]] == 'free':
                    self.analog_pins[f'arduino_{clients[client]["arduino"]}'][
                        clients[client]["pin"]] = f'{arm_name} - {client}'
                else:
                    self.occupied_pin(f'Analog pin #{clients[client]["pin"]} on arduino #{clients[client]["arduino"]}',
                                      f'{arm_name} - {client}')

            if pin_type == 'digital':
                if self.digital_pins[clients[client]] == 'free':
                    self.digital_pins[clients[client]] = f'{arm_name} - {client}'
                else:
                    self.occupied_pin(f'Digital pin #{clients[client]}', f'{arm_name} - {client}')

        return clients

    def occupied_pin(self, pin_to_assign, client_to_assign):
        a = self.analog_pins
        print(f"{pin_to_assign} is already in use, can't assign to {client_to_assign}, the program will shut down.")
        exit()


egse = EGSE({'arm1': {"arduino": 0,
                      "limit_switches": {'limit_switch_1': 3, 'limit_switch_2': 5, 'limit_switch_3': 7},
                      "hdrms": {'hdrm_1': 8},
                      "motor": {'motor_direction': 10, 'motor_step': 11},
                      "strain_gages": {'strain_gage_1': {'arduino': 0, 'pin': 0},
                                       'strain_gage_2': {'arduino': 0, 'pin': 1},
                                       'strain_gage_3': {'arduino': 0, 'pin': 2}},
                      "thermistors": {'thermistor_1': {'arduino': 0, 'pin': 3},
                                      'thermistor_2': {'arduino': 0, 'pin': 4},
                                      'thermistor_3': {'arduino': 0, 'pin': 5}}},

             'arm2': {"arduino": 0,
                      "limit_switches": {'limit_switch_1': 12, 'limit_switch_2': 13, 'limit_switch_3': 15},
                      "hdrms": {'hdrm_1': 16},
                      "motor": {'motor_direction': 18, 'motor_step': 19},
                      "strain_gages": {'strain_gage_1': {'arduino': 0, 'pin': 6},
                                       'strain_gage_2': {'arduino': 0, 'pin': 7},
                                       'strain_gage_3': {'arduino': 0, 'pin': 8}},
                      "thermistors": {'thermistor_1': {'arduino': 0, 'pin': 9},
                                      'thermistor_2': {'arduino': 0, 'pin': 10},
                                      'thermistor_3': {'arduino': 0, 'pin': 11}}},

             'arm3': {"arduino": 1,
                      "limit_switches": {'limit_switch_1': 21, 'limit_switch_2': 22, 'limit_switch_3': 23},
                      "hdrms": {'hdrm_1': 24},
                      "motor": {'motor_direction': 26, 'motor_step': 27},
                      "strain_gages": {'strain_gage_1': {'arduino': 1, 'pin': 0},
                                       'strain_gage_2': {'arduino': 1, 'pin': 1},
                                       'strain_gage_3': {'arduino': 1, 'pin': 2}},
                      "thermistors": {'thermistor_1': {'arduino': 1, 'pin': 3},
                                      'thermistor_2': {'arduino': 1, 'pin': 4},
                                      'thermistor_3': {'arduino': 1, 'pin': 5}}},

             'arm4': {"arduino": 1,
                      "limit_switches": {'limit_switch_1': 28, 'limit_switch_2': 29, 'limit_switch_3': 31},
                      "hdrms": {'hdrm_1': 32},
                      "motor": {'motor_direction': 33, 'motor_step': 35},
                      "strain_gages": {'strain_gage_1': {'arduino': 1, 'pin': 6},
                                       'strain_gage_2': {'arduino': 1, 'pin': 7},
                                       'strain_gage_3': {'arduino': 1, 'pin': 8}},
                      "thermistors": {'thermistor_1': {'arduino': 1, 'pin': 9},
                                      'thermistor_2': {'arduino': 1, 'pin': 10},
                                      'thermistor_3': {'arduino': 1, 'pin': 11}}},
             })
arm1_state = egse.arm1.get_arm_state()
for sensor_group in arm1_state:
    print(f'\n{sensor_group}:')
    for sensor in arm1_state[sensor_group]:
        space = [' ' for i in range(len(sensor_group) + 2)]
        print(f'{"".join(space)}{sensor}: {arm1_state[sensor_group][sensor]}')

print('1')
