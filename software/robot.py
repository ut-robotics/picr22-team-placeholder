from motion import IRobotMotion
import serial
import struct


class AwesomeRobot(IRobotMotion):
    def __init__(self, port):
        self.port = port
        self.serial = serial.Serial(port)

    def open(self):
        print("AwesomeRobot starting....")

    def close(self):
        print("Gonna kms now :^)!")
        self.serial.close()

    # speedx3 int16_t
    # throwerSpeed uint16_t
    # disableFailSafe 1 or else / uint8_t
    # delimiter uint16_t
    # the base of this code was written by rasmus
    def send(self, speed1, speed2, speed3, throwerSpeed, disableFailsafe=0, delimiter=0xAAAA):
        """         if (speed1 < 0):
            speed1 += 2 ** 16
        if (speed2 < 0):
            speed2 += 2 ** 16
        if (speed3 < 0):
            speed3 += 2 ** 16 """
        print(struct.pack('<hhhHBH', speed1, speed2, speed3, throwerSpeed, disableFailsafe, delimiter))
        self.serial.write(struct.pack('<hhhHBH', speed1, speed2, speed3, throwerSpeed, disableFailsafe, delimiter))
        received_data = self.serial.read(8)
        actual_speed1, actual_speed2, actual_speed3, feedback_delimiter = struct.unpack('<hhhH', received_data)
        print(actual_speed1, actual_speed2, actual_speed3)
        # i hate python - Artur



if __name__ == "__main__":
    robot = AwesomeRobot('/dev/ttyACM0')
    for i in range(5):
        robot.send(-16, -16, -16, 0)
