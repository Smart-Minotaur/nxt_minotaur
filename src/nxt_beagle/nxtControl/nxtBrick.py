#!/usr/bin/env python

# Author: Fabian Meyer

import nxt.locator
import nxt.motor
import nxt.sensor

class BrickController:
    "This class is used to control a NXT-Brick."
    
    left_motor_port = nxt.motor.PORT_A
    right_motor_port = nxt.motor.PORT_B
    
    id_left = 0
    id_right = 1
    
    __brick = None
    __ultrasonic_sensors = []
    __motors = []
    __sensor_ports = { 1 : nxt.sensor.PORT_1,
                       2 : nxt.sensor.PORT_2,
                       3 : nxt.sensor.PORT_3,
                       4 : nxt.sensor.PORT_4 }
    
    
    def __init__(self):
        pass
        
    def connectToBrick(self, brick_name=None):
        "connects to brick and adds motors"
        self.__motors[:] = []
        self.__ultrasonic_sensors[:] = []
        # configure brick
        self.__brick = nxt.locator.find_one_brick(name=brick_name)
        self.__motors.append(nxt.motor.Motor(self.__brick, self.left_motor_port))
        self.__motors.append(nxt.motor.Motor(self.__brick, self.right_motor_port))
        
        
    def setMotorPower(self, motor_id, motor_power):
        if(motor_power == 0):
            self.__motors[motor_id].brake()
        else:
            self.__motors[motor_id].run(motor_power)
        
    def setMotors(self, left_power, right_power):
        self.setMotorPower(self.id_left, left_power)
        self.setMotorPower(self.id_right, right_power)
    
    def getMotorTicks(self, motor_id):
        return self.__motors[motor_id].get_tacho()
        
    def addUltrasonicSensor(self, port):
        if (port > 4) or (port < 1):
            raise ValueError("Incorrect port for UltrasonicSensor.")
        self.__ultrasonic_sensors.append(nxt.sensor.Ultrasonic(self.__brick, self.__sensor_ports[port]))
        return len(self.__ultrasonic_sensors) - 1
    
    def getUltrasonicData(self, sensor_id):
        return self.__ultrasonic_sensors[sensor_id].get_distance()
        
    def resetMotorTicks(self):
        for motor in self.__motors:
            motor.reset_position(True)
            
    def clearSensors(self):
        self.__ultrasonic_sensors[:] = []
        