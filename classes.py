import RPi.GPIO as GPIO
import time


GPIO.setmode(GPIO.BCM)


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
                'current_position': self.current_position,
                'limit_switch': self.limit_switch_state}

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
        return self.self.transform_function(analog_data)

    def read_sensor(self, analog_data):
        return self.transform(analog_data[self.sensor_type][self.arduino_address])


class DigitalSensor:
    def __init__(self, sensor_pin):
        self.sensor_pin = sensor_pin
        self.sensor_setup()
        self.state = self.check_state()

    def sensor_setup(self):
        GPIO.setup(self.sensor_pin, GPIO.IN)

    def check_state(self):
        self.state = GPIO.input(self.sensor_pin)
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
                 thermistor_transform_function, hdrm_pin_kwargs):
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

    def get_arm_state(self, arduino_data):
        return {'limit_switches': self.limit_switches.get_values(),
                'motor': self.motor.get_values(),
                'strain_gages': self.strain_gages.get_values(arduino_data),
                'thermistors': self.thermistors.get_values(arduino_data),
                'hdrm': self.hdrm.get_values()}


arm = Arm(limit_switches_kwargs={'limit_switch_1': 11, 'limit_switch_2': 12, 'limit_switch_3': 13},
          motor_name='first motor',
          motor_direction_pin=15,
          motor_step_pin=16,
          motor_direction='open',
          motor_step_resolution=0.05,
          strain_gages_kwargs={'strain_gage_1': 17, 'strain_gage_2': 18, 'strain_gage_3': 19},
          strain_gage_transform_function=0,
          thermistors_kwargs={'thermistor_1': 20, 'thermistor_2': 21, 'thermistor_3': 22},
          thermistor_transform_function=0,
          hdrm_pin_kwargs={'hdrm_1': 23})

print('a')
