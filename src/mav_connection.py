import socket
import threading
import time
import struct
from src.mav_message import MAVLinkChecksum


# MAVLink constants
START_BYTE = 0xFE
SYSTEM_ID = 0
COMPONENT_ID = 1
SEQUENCE = 0

BUFFER_SIZE = 1024


class MAVLinkSocket:
    def __init__(self, host: str, port: int, broadcast_port: int):
        self.host = host
        self.port = port
        self.broadcast_port = broadcast_port
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind((self.host, self.port))
        self.gcs_ip = None
        self.gcs_port = None
        self.access_control = AccessControl()

    def broadcast_address(self):
        broadcast_address = ('<broadcast>', self.broadcast_port)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        authorized_system_ids = [0, 5]
        message = f"{authorized_system_ids}".encode()
        self.udp_socket.sendto(message, broadcast_address)

    def recv_datagram(self):
        if self.udp_socket:
            data, addr = self.udp_socket.recvfrom(BUFFER_SIZE)
            system_id = self.parse_system_id(data)
            if self.access_control.is_authorized(system_id):
                return data, addr
            else:
                print(f"Access denied for system ID {system_id}")
                return None, None
        else:
            raise ConnectionError("UDP socket is not connected")

    def parse_system_id(self, data: bytes):
        sys_id = data[3]
        if not self.access_control.has_authorized_ids():
            self.access_control.configure_access([sys_id])
            return sys_id
        elif self.access_control.is_authorized(sys_id):
            return sys_id
        else:
            return None


class MAVLinkSocketHandler:
    def __init__(self, bind_ip, bind_port, remote_port):
        self.drone_socket = MAVLinkSocket(bind_ip, bind_port, remote_port)
        self.broadcasting = True
        self.stop_event = threading.Event()
        self.listen_thread = threading.Thread(target=self.listen_for_datagrams)
        self.broadcast_thread = threading.Thread(target=self.broadcast_address)

    def start(self):
        self.listen_thread.start()
        self.broadcast_thread.start()
        self.listen_thread.join()
        self.broadcast_thread.join()

    def broadcast_address(self):
        while not self.stop_event.is_set():
            self.drone_socket.broadcast_address()
            print("Sent broadcast message")
            time.sleep(2)

    def listen_for_datagrams(self):
        while True:
            data, addr = self.drone_socket.recv_datagram()
            if data:
                print("Received datagram:", data, "from", addr)
                self.receive_message(data)
                self.stop_event.set()
                self.broadcasting = False

    @staticmethod
    def receive_message(data):
        print(f"Bytes Received: {len(data)}")

        hex_string = ' '.join(format(byte, '02x') for byte in data)
        print(f"Datagram: {hex_string}")

        if len(data) < 8:
            print("Invalid packet length")
            return

        start_byte, length, sequence, system_id, component_id, msg_id = struct.unpack('<BBBBBB', data[:6])

        if start_byte == START_BYTE:
            payload = data[6:6 + length]
            received_checksum = data[6+length]

            computed_checksum = MAVLinkChecksum.compute(data[1:6 + length])

            if received_checksum == computed_checksum:
                print(f"Received packet: SYS: {system_id}, COMP: {component_id}, LEN: {length}, MSG ID: {msg_id}, "
                      f"Payload: {payload}")
                return
            else:
                print(f"Invalid checksum: received {received_checksum}, computed {computed_checksum}")
                return
        else:
            print("Invalid MAVLink message start byte")
            return None


class AccessControl:
    def __init__(self):
        self.authorized_system_ids = set()

    def configure_access(self, system_ids: list[int]):
        self.authorized_system_ids.update(system_ids)

    def is_authorized(self, system_id: int) -> bool:
        return system_id in self.authorized_system_ids

    def has_authorized_ids(self) -> bool:
        return len(self.authorized_system_ids) > 0
