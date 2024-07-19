from src.mav_connection import MAVLinkSocketHandler

if __name__ == "__main__":
    handler = MAVLinkSocketHandler('0.0.0.0', 50000, 50001)
    handler.start()
