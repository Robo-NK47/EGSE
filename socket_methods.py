import pickle


def talk(_socket, data):
    data_to_server = pickle.dumps(data)
    data_to_server = bytes(f"{len(data_to_server):<{header_size}}", 'utf-8') + data_to_server
    _socket.send(data_to_server)
    print(f'Message length: {len(data_to_server)}')


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


header_size = 10
