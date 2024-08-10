import time
from src.mav_connection import MAVLinkSocketHandler
from src.mav_message import MAVLinkMessageCreator


status_values = {
    'onboard_control_sensors_health': 1,  # MAV_SYS_STATUS_SENSOR: Showing onboard controllers and sensors have error
    'load': 500,  # Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000
    'current_battery': 30,  # Battery current, -1: Current not sent by autopilot
    'battery_remaining': 45  # Battery remaining energy not sent by autopilot
}


class Drone:
    def __init__(self, latitude, longitude):
        self.alive = False
        self.onboard_control_sensors_present = 1
        self.load = 500
        self.current_battery = 30
        self.battery_remaining = 45
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = None
        self.handler = None

    def start_drone(self):
        self.handler = MAVLinkSocketHandler('0.0.0.0', 50000, 50001, self)
        try:
            self.handler.start()
        except KeyboardInterrupt:
            print("Program interrupted. Exiting.")
        finally:
            print("Drone shutting down")

    @staticmethod
    def set_drone_active(self_instance):
        if isinstance(self_instance, Drone):
            self_instance.alive = True
            print("Its awake now")
        else:
            raise ValueError("Argument must be of type Drone")

    def set_location(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude
        print(f"Drone moved to Latitude: {self.latitude}, Longitude: {self.longitude}")

