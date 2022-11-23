from serial import Serial
import socket
import socket_methods
import struct


def get_data_from_arduino():
    while True:
        if arduino.inWaiting() > 0:
            current_data = arduino.read(4)
            return struct.unpack('<f', current_data)[0]


def read_all_analog_signals():
    analog_sensors = {'strain_gages': 2.0, 'thermistors': 3.0}
    for sensor in analog_sensors:
        while not get_data_from_arduino() == analog_sensors[sensor]:
            pass
        _sensor = []
        for _ in range(8):
            _sensor.append(get_data_from_arduino())
        analog_sensors[sensor] = _sensor

    return analog_sensors


def create_reliable_connection(_com, _baud_rate):
    global arduino
    large_counter = 1
    while True:
        arduino = Serial(_com, _baud_rate)
        counter = 0
        while counter < 100:
            data = get_data_from_arduino()
            if data == 2.0:
                print(f'Successful connection after {large_counter} attempts.')
                return arduino
            counter += 1
        large_counter += 1


arduino_COM = '/dev/ttyACM0'
baud_rate = 115200
arduino = create_reliable_connection(arduino_COM, baud_rate)


if __name__ == '__main__':
    header_size = 10
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host = socket.gethostname()
    print(f'\n\nEnter "{host}" in the host variable in the client side.')
    sock.bind((host, 1243))
    sock.listen(5)

    print('Waiting for client.')
    client_socket, address = sock.accept()

    while True:
        socket_methods.talk(client_socket, read_all_analog_signals())
        print(read_all_analog_signals())
