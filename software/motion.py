import turtle
import math
import numpy as np
import time
import tkinter as tk
import struct
import serial


class IRobotMotion:
    def open(self):
        pass

    def close(self):
        pass

    def move(self, x_speed, y_speed, rot_speed):
        pass


class OmniRobot(IRobotMotion):
    # initializee
    def __init__(self):
        gearboxReductionRatio = 18.75
        encoderEdgesPerMotorRevolution = 64
        wheelRadius = 0.035  # metres
        pidControlFrequency = 100  # Hz
        self.wheelAngles = [0, 120, 240]
        self.wheelSpeedToMainboardUnits = gearboxReductionRatio * \
            encoderEdgesPerMotorRevolution / \
            (2 * math.PI * wheelRadius * pidControlFrequency)
        self.wheelDistanceFromCenter = 0.15  # TODO - verify

    # opening the serial connection
    def open(self):
        self.ser = serial.Serial()
        self.ser.port = '/dev/ttyACM0'
        self.ser.baudrate = 9600
        self.ser.open()
        print("Serial opened")

    # closing serial connection
    def close(self):
        self.stop()
        self.ser.close()
        print("Serial closed")

    # writing and reading to and from serial

    def send(self, speeds, throwerSpeed, disableFailsafe=0, delimiter=0xAAAA):
        # Tere Timo ja Jens!!!!!!!!!!
        sent_data = struct.pack(
            '<hhhHBH', speeds[0], speeds[1], speeds[2], throwerSpeed, disableFailsafe, delimiter)
        self.serial.write(sent_data)
        print("sent:", sent_data)
        received_data = self.serial.read(8)
        actual_speed1, actual_speed2, actual_speed3, _ = struct.unpack(
            '<hhhH', received_data)
        print("received", actual_speed1, actual_speed2, actual_speed3)
        # i hate python - Artur

    # wheel speed calculation
    def get_wheel_speed(self, motorID, robotSpeed, robotDirectionAngle, robotAngularVelocity):
        wheelLinearVelocity = robotSpeed * \
            math.cos(robotDirectionAngle -
                     self.wheelAngles[motorID]) + self.wheelDistanceFromCenter * robotAngularVelocity
        wheelAngularSpeedInMainboardUnits = wheelLinearVelocity * \
            self.wheelSpeedToMainboardUnits
        return wheelAngularSpeedInMainboardUnits

    # movement calculation + sending
    def move(self, x_speed, y_speed, rot_speed, thrower_speed=0):
        robot_speed = math.sqrt(x_speed**2 + y_speed**2)
        robot_angle = math.atan2(y_speed, x_speed)  # verify
        speeds = [int(self.get_wheel_speed(i, robot_speed,
                      robot_angle, rot_speed)) for i in range(3)]
        print("Speeds:", speeds)
        self.send(speeds, thrower_speed)

    def stop(self):
        self.move(0, 0, 0)

    # for testing motors
    def motor_test(self, thrower=0):
        self.send([10, 10, 10], thrower)
