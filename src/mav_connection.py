import socket
import threading
import time
import struct

from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.backends import default_backend
import os

from src.mav_message import MAVLinkChecksum, MAVLinkSerializer, MAVLinkMessageCreator, MAVLinkMessage

# MAVLink constants
START_BYTE = 0xFE
SYSTEM_ID = 1  # Can't use Zero as it is the broadcast address
COMPONENT_ID = 1
SEQUENCE = 0

BUFFER_SIZE = 1024

heartbeat_values = {
    'type': 2,  # MAV_TYPE: Generic micro air vehicle
    'autopilot': 1,  # MAV_AUTOPILOT: Reserved for future use.
    'base_mode': 0,  # MAV_MODE_FLAG: (Bitmask) These flags encode the MAV mode.
    'custom_mode': 1,  # A bitfield for use for autopilot-specific flags
    'system_status': 1,  # MAV_STATE: System status flag.
    'mavlink_version': 3  # MAVLink version
}

CRC_EXTRA_CONSTANTS = {
    0: 50  # HEARTBEAT
}

STATIC_KEY = None


class MAVLinkSocket:
    def __init__(self, host: str, port: int, broadcast_port: int):
        self.host = host
        self.port = port
        self.broadcast_port = broadcast_port
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind((self.host, self.port))
        self.access_control = AccessControl()

    def broadcast_address(self):
        broadcast_address = ('<broadcast>', self.broadcast_port)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        message = f'{SYSTEM_ID}'.encode('utf-8')
        self.udp_socket.sendto(message, broadcast_address)

    def send_datagram(self, data: bytes, target_host: str, target_port: str) -> None:
        try:
            if self.udp_socket:
                self.udp_socket.sendto(data, (target_host, target_port))
        except socket.error as e:
            print(f"Socket error: {e}")

    def recv_datagram(self):
        if self.udp_socket:
            data, addr = self.udp_socket.recvfrom(BUFFER_SIZE)
            if STATIC_KEY is None:
                self.parse_static_key(data)
                return data, addr
            else:
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

    @staticmethod
    def parse_static_key(data: bytes):
        start_byte = data[0]
        global STATIC_KEY
        if start_byte == 1:
            static_key = data[1:]
            STATIC_KEY = static_key


class MAVLinkSocketHandler:
    def __init__(self, bind_ip, bind_port, remote_port, drone):
        self.drone_socket = MAVLinkSocket(host=bind_ip, port=bind_port, broadcast_port=remote_port)
        self.broadcasting = True
        self.listen_thread = threading.Thread(target=self.listen_for_datagrams)
        self.heartbeat_thread = None
        self.gcs_ip = None
        self.gcs_port = None
        self.drone = drone

    def start(self):
        self.listen_thread.start()
        self.broadcast_address()
        try:
            self.listen_thread.join()
        except KeyboardInterrupt:
            print("Stopping Listening.")

    def broadcast_address(self):
        while self.broadcasting:
            try:
                self.drone_socket.broadcast_address()
                print("Sent broadcast message")
                time.sleep(2)
            except Exception as e:
                print(f"Error sending broadcast: {e}")

    def enable_drone(self):
        from src.Drone_sim import Drone
        # Setting the drone to alive
        Drone.set_drone_active(self.drone)
        if not self.heartbeat_thread or not self.heartbeat_thread.is_alive():
            self.heartbeat_thread = threading.Thread(target=self.send_heartbeat)
            self.heartbeat_thread.start()

    def listen_for_datagrams(self):
        operations_executed = False

        while True:
            data, addr = self.drone_socket.recv_datagram()
            self.gcs_ip, self.gcs_port = addr
            if data and (data[0] == START_BYTE):
                print("Received datagram:", data, "from", addr)
                self.receive_message(data)
                if not operations_executed:
                    self.enable_drone()
                    self.broadcasting = False
                    operations_executed = True
                    print("Stopping broadcast")

    def send_message(self, message: MAVLinkMessage, values: dict, encrypted) -> None:
        serializer = MAVLinkSerializer(message)
        payload = serializer.serialize(values)

        mav_message = payload

        if encrypted:
            # Encrypt message
            encryptor = MAVLinkSec(STATIC_KEY)
            mav_message = encryptor.encrypt_chacha20(payload)

        # Packet creation
        length = len(mav_message)
        msg_id = message.message_id
        header = struct.pack('<BBBBBB', START_BYTE, length, SEQUENCE, SYSTEM_ID, COMPONENT_ID, msg_id)

        # Compute Checksum
        # Compute Checksum
        crc_extra = CRC_EXTRA_CONSTANTS.get(msg_id, 0)
        checksum = MAVLinkChecksum().compute(header[1:] + mav_message, crc_extra)
        checksum_bytes = struct.pack('<H', checksum)

        mavlink_packet = header + mav_message + checksum_bytes

        print("Bytes Sent:", len(mavlink_packet))

        hex_string = ' '.join(format(byte, '02x') for byte in mavlink_packet)
        print("Datagram:", hex_string)
        self.drone_socket.send_datagram(mavlink_packet, self.gcs_ip, self.gcs_port)
        print(f"Sent to address:{self.gcs_ip}:{self.gcs_port}")

    def send_heartbeat(self):
        mav_message = MAVLinkMessageCreator().create_message(0)
        while self.drone.alive is True:
            try:
                if STATIC_KEY is None:
                    self.send_message(mav_message, heartbeat_values, False)
                    time.sleep(30)
                    print("Heartbeat sent")
                else:
                    self.send_message(mav_message, heartbeat_values, True)
                    time.sleep(30)
                    print("Heartbeat sent")
            except Exception as e:
                print(f"Error sending heartbeat: {e}")

    @staticmethod
    def receive_message(data):
        if len(data) < 8:
            print("Invalid packet length")
            return

        start_byte, length, sequence, system_id, component_id, msg_id = struct.unpack('<BBBBBB', data[:6])

        if start_byte == START_BYTE:
            print(f"Bytes Received: {len(data)}")

            hex_string = ' '.join(format(byte, '02x') for byte in data)
            print(f"Datagram: {hex_string}")

            mav_message = MAVLinkMessageCreator().create_message(msg_id)

            if mav_message is None:
                print("Message ID not recognized")
                return

            received_payload = data[6:6 + length]
            # Decrypt message
            if length > 10:  # if length is greater than 10 for now, it means that the message is encrypted
                decryptor = MAVLinkSec(STATIC_KEY)
                received_payload = decryptor.decrypt_chacha20(data[6:6 + length])

            serializer = MAVLinkSerializer(mav_message)
            payload = serializer.deserialize(received_payload)
            received_checksum = data[6 + length]

            crc_extra = CRC_EXTRA_CONSTANTS.get(msg_id, 0)
            checksum_data = data[1:6 + length]
            computed_checksum = MAVLinkChecksum().compute(checksum_data, crc_extra)

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


class MAVLinkSec:
    def __init__(self, key=STATIC_KEY):
        self.key = key

    def encrypt_chacha20(self, payload):
        nonce = os.urandom(16)

        algorithm = algorithms.ChaCha20(self.key, nonce)
        cipher = Cipher(algorithm, mode=None, backend=default_backend())
        encryptor = cipher.encryptor()
        ciphertext = encryptor.update(payload)

        return nonce + ciphertext

    def decrypt_chacha20(self, encrypted_message):
        nonce = encrypted_message[:16]
        ciphertext = encrypted_message[16:]

        algorithm = algorithms.ChaCha20(self.key, nonce)
        cipher = Cipher(algorithm, mode=None, backend=default_backend())
        decryptor = cipher.decryptor()
        decrypted_message = decryptor.update(ciphertext)

        return decrypted_message

    def aes_encrypt(self, payload):
        nonce = os.urandom(16)

        algorithm = algorithms.AES(self.key)
        cipher = Cipher(algorithm, modes.CTR(nonce), backend=default_backend())

        encryptor = cipher.encryptor()
        ciphertext = encryptor.update(payload) + encryptor.finalize()

        return nonce + ciphertext

    def aes_decrypt(self, cipher_message):
        nonce = cipher_message[:16]
        encrypted_message = cipher_message[16:]

        algorithm = algorithms.AES(self.key)
        cipher = Cipher(algorithm, modes.CTR(nonce), backend=default_backend())

        decrypt = cipher.decryptor()
        decrypted_message = decrypt.update(encrypted_message) + decrypt.finalize()

        return decrypted_message
