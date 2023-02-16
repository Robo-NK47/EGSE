import pickle
import socket


def talk(_socket, data):
    data_to_server = pickle.dumps(data)
    data_to_server = bytes(f"{len(data_to_server):<{header_size}}", 'utf-8') + data_to_server
    _socket.send(data_to_server)
    # print(f'Message length: {len(data_to_server)}')


def listen(_socket, number_of_bytes):
    full_msg = b''
    new_msg = True

    while True:
        msg = _socket.recv(number_of_bytes)
        if new_msg:
            msglen = int(msg[:header_size])
            new_msg = False

        full_msg += msg
        if len(full_msg) - header_size == msglen:
            data = pickle.loads(full_msg[header_size:])
            new_msg = True
            full_msg = b""

        if new_msg:
            return data


def is_socket_in_use(address, port):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((address, port))
    except OSError as e:
        if e.errno == 98:
            return True
        else:
            raise
    finally:
        s.close()
    return False


def create_host(address, port):
    if not is_socket_in_use(address, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = socket.gethostname()
        print(f'\n\nEnter "{host}" in the host variable in the client side.')
        sock.bind((host, 1243))
        return sock

    else:
        print(f'Port {port} in address {address} is in use.')
        exit()


# def connect_to_host(host):
#


header_size = 10
