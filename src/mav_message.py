import xml.etree.ElementTree as ET
import struct


class MAVLinkField:
    def __init__(self, field_type: str, name: str, description: str):
        self.field_type = field_type
        self.name = name
        self.description = description


class MAVLinkMessage:
    def __init__(self, message_id: int, fields: list[MAVLinkField]):
        self.message_id = message_id
        self.fields = fields


class MAVLinkXMLParser:
    def parse_file(self, file_path: str) -> list[MAVLinkMessage]:
        tree = ET.parse(file_path)
        root = tree.getroot()
        messages = []
        for message_element in root.find('messages').findall('message'):
            messages.append(self.parse_msg(message_element))
        return messages

    def parse_msg(self, message_element) -> MAVLinkMessage:
        msg_id = int(message_element.attrib['id'])
        fields = [self.parse_field(field) for field in message_element.findall('field')]
        return MAVLinkMessage(msg_id, fields)

    @staticmethod
    def parse_field(field_element) -> MAVLinkField:
        field_type = field_element.attrib['type']
        name = field_element.attrib['name']
        description = field_element.text
        return MAVLinkField(field_type, name, description)


class MAVLinkSerializer:
    def __init__(self, message: MAVLinkMessage):
        self.message = message

    def serialize(self, values: dict) -> bytes:
        format_string = '<' + ''.join([self._get_format(f.field_type) for f in self.message.fields])
        payload = struct.pack(format_string, *[values[f.name] for f in self.message.fields])
        return payload

    def deserialize(self, payload: bytes):
        format_string = '<' + ''.join([self._get_format(f.field_type) for f in self.message.fields])
        unpacked_values = struct.unpack(format_string, payload)
        return {f.name: value for f, value in zip(self.message.fields, unpacked_values)}

    # Got this from https://mavlink.io/en/guide/serialization.html
    @staticmethod
    def _get_format(field_type: str) -> str:
        formats = {
            'uint8_t': 'B',
            'uint8_t_mavlink_version': 'B',
            'int8_t': 'b',
            'uint16_t': 'H',
            'int16_t': 'h',
            'uint32_t': 'I',
            'int32_t': 'i',
            'float': 'f',
            'char': 'c'
        }
        return formats[field_type]


class MAVLinkMessageCreator:
    def __init__(self):
        self.parser = MAVLinkXMLParser()

    def create_message(self, msg_id):
        messages = self.parser.parse_file('message_definitions/common.xml')
        mav_message = next(mav_message for mav_message in messages if mav_message.message_id == msg_id)
        return mav_message


class MAVLinkChecksum:
    def __init__(self, message: MAVLinkMessage):
        self.message = message

    @staticmethod
    def compute(data: bytes) -> int:
        checksum = sum(data) % 256
        return checksum
