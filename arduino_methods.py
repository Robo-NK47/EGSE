import serial


def check_which_port():
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    ports_list = []
    for p in ports:
        ports_list.append(p)
    return ports_list


def get_arduino_ports():
    ports = check_which_port()
    arduino_devices = []
    for p in ports:
        if 'ttyACM' in p.description:
            arduino_devices.append(p.device)
    if len(arduino_devices) > 0:
        return arduino_devices
    return None


def get_data_from_arduino(arduino):
    while True:
        if arduino.inWaiting() > 0:
            try:
                line = arduino.readline().decode()
                try:
                    return int(line)
                except ValueError:
                    pass
            except UnicodeDecodeError:
                pass


def get_all_analog_values(arduino, _header_value):
    header = get_data_from_arduino(arduino)
    while header != _header_value:
        header = get_data_from_arduino(arduino)
        pass
    analog_values = {'header': header}
    for sensor_number in range(16):
        analog_values[sensor_number] = get_data_from_arduino(arduino)
    return analog_values


def generate_arduinos(baud_rate=115200, header_value=1024):
    _arduinos = {}
    for i, arduino_port in enumerate(get_arduino_ports()):
        _arduinos[f'arduino {i}'] = serial.Serial(arduino_port, baud_rate)

    return _arduinos, header_value


if __name__ == "main":
    arduinos, header = generate_arduinos()
    while True:
        for arduino in arduinos:
            value = get_all_analog_values(arduinos[arduino], header)
            print(f'{arduino} - {value}')
        print("")
