import grovepi
import brickpi3
import sys
from time import sleep, time
from math import copysign, sqrt
from MPU9250 import MPU9250
from IMUFilters import *
import numpy as np

BP = brickpi3.BrickPi3()
IMU = MPU9250()

BPPort = {1:BP.PORT_1, 2:BP.PORT_2, 3:BP.PORT_3, 4:BP.PORT_4, 
                "A":BP.PORT_A,"B":BP.PORT_B, "C":BP.PORT_C, "D":BP.PORT_D}

basePower = 65
reduce = basePower * 0.8
maxPower = 90
period = 0.02 # seconds
g = 9.81 #m/s^2


# -------------------- Sensor setup --------------------

class sensorType():
    grove = 0
    brick = 1
    analog = 3
    digital = 4
    ultraSonic = 5
    hall = 6
    colorReflect = BP.SENSOR_TYPE.EV3_COLOR_REFLECTED
    colorAmbient = BP.SENSOR_TYPE.EV3_COLOR_AMBIENT
    gyro = BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS
    IMU = 7
    motor = 8
    

class Sensor():

    def __init__(self, type, mode, pin, biased = False, bias = 0, multiplier = 1) -> None:
        self.type = type
        self.mode = mode
        self.pin = pin
        self.value = 0
        self.biased = biased
        self.bias = bias
        self.multiplier = multiplier
        if(type == sensorType.grove): grovepi.pinMode(pin, "INPUT")
        if (type == sensorType.brick): BP.set_sensor_type(BPPort[pin], self.mode)
    
    def update(self) -> float:
        if(self.type == sensorType.grove): 
            if(self.mode == sensorType.analog):       self.value = grovepi.analogRead(self.pin)
            elif(self.mode == sensorType.digital):    self.value = grovepi.digitalRead(self.pin)
            elif(self.mode == sensorType.ultraSonic): self.value = grovepi.ultrasonicRead(self.pin)
            elif(self.mode == sensorType.hall): self.value = not grovepi.digitalRead(self.pin)
        elif (self.type == sensorType.brick):
            if(self.mode == sensorType.motor): self.value = BP.get_motor_encoder(BPPort[self.pin])
            else:                              self.value = BP.get_sensor(BPPort[self.pin])


        if self.biased: self.value = (self.value + self.bias) * self.multiplier
        return self.value

class IMUWrap():

    def __init__(self) -> None:
        self.accelBias : np.ndarray = None
        self.gyroBias  : np.ndarray = None
        self.magBias   : np.ndarray = None

        self.acceleration : np.ndarray = None
        self.gyro         : np.ndarray = None
        self.magnetic     : np.ndarray = None

        self.magThreshold = 0

        self.velocity = np.array() # meters/sec
        self.position = np.array() # meters

    def getBias(self):
        biases = AvgCali(IMU, 100, 0.04)
        # splits biases into 3 sublists and assigns them accordingly
        self.accelBias, self.gyroBias, self.magBias = [np.array(biases[x: x+3]) for x in range(0, len(biases), 3)]

    def update(self, kinematics = False):
        self.acceleration = np.array([float(x) for x in IMU.readAccel().values()])  - self.accelBias
        self.gyro         = np.array([float(x) for x in IMU.readGyro().values()])   - self.gyroBias
        self.magnetic     = np.array([float(x) for x in IMU.readMagnet().values()]) - self.magBias

        if kinematics: self.kinematics()
    
    @property
    def magneticMagnitude(self) -> float:
        return np.sqrt((self.magnetic**2).sum())
    
    @property
    def velocityCm(self) -> float: # returns the velocity in cm
        return self.velocity * 100
    
    @property
    def positionCm(self) -> float: # returns the position in cm
        return self.position * 100

    def onBeacon(self) -> bool:
        return self.magneticMagnitude > self.magThreshold

    def kinematics(self):
        acceleration = self.acceleration * g
        self.velocity += acceleration * period
        self.position += self.velocity * period





# -------------------- Motor setup --------------------

class Motor():

    def __init__(self, pin, power = 0, reverse = False) -> None:
        self.power = power
        self.reverse = reverse
        self.pin = BPPort[pin]
        BP.offset_motor_encoder(self.pin, BP.get_motor_encoder(self.pin))
    
    def setPower(self, power):
        self.power = power
    
    def goToPosition(self, position):
        BP.set_motor_position(self.pin, position)

    def update(self):
        BP.set_motor_power(self.pin, self.power - 2 * self.power * self.reverse)

    def getPosition(self):
        BP.get_motor_encoder(BPPort[self.pin])


class DriveTrain():                                                                 

    def __init__(self, leftpin, rightpin) -> None:
        self.left  = Motor(leftpin)
        self.right = Motor(rightpin)

    def setAllPowers(self, power):
        self.left.setPower(power)
        self.right.setPower(power)
    
    def reduceLeft(self, reduce):
        self.left.power -= copysign(reduce, self.left.power)

    def reduceRight(self, reduce):
        self.right.power -= copysign(reduce, self.right.power)

    def setLeft(self, power):
        self.left.setPower(power)

    def setRight(self, power):
        self.right.setPower(power)

    def turn(self, direction : bool, power):
        # :param direction: True = Right, False = left
        self.setRight(power - 2 * power * direction)
        self.setLeft(power - 2 * power * (not direction))
    
    def update(self):
        self.left.update()
        self.right.update()

# -------------------- PID --------------------

class PIDController():

    def __init__(self, Kp, Ki, Kd, sensor, target = 0, derivativeMax = 30) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.sensor = sensor
        self.target = target

        self.error      = 0
        self.lastError  = 0
        self.totalError = 0

        self.derivativeMax = derivativeMax

        self.partialOut    = 0
        self.integralOut   = 0
        self.derivativeOut = 0
        self.output        = 0

    # --------------- update section ---------------
    def update(self, interest = "", verbose = False):
        self.lastError = self.error
        # if self.sensor.__class__ is IMUWrap:
        #     if interest.lower == "p": self.error = self.target - self.sensor.positionCm
        #     else: self.error = self.target - self.sensor.velocityCm
        # else:
        self.error = self.target - self.sensor.update()


        self.totalError += self.error * period * (abs(self.integralOut) <= maxPower)
        self.setOutput()
        if verbose:
            print(f"Current Error:     {self.error:.2f}")
            print(f"Partial output:    {self.partialOut:.2f}")
            print(f"Integral output:   {self.integralOut:.2f}")
            print(f"Derivative output: {self.derivativeOut:.2f}")
            print(f"Output:            {self.output:.2f}\n")
        
    
    def setOutput(self):
        self.partialOut    = self.error * self.Kp
        self.integralOut   = self.totalError * self.Ki
        self.derivativeOut = self.Kd*(self.error - self.lastError)/period

        self.output = min([self.partialOut + self.integralOut + (self.derivativeOut * (abs(self.derivativeOut) <= self.derivativeMax)), maxPower])






def delay():
    input("Press Enter when ready")
    print("------  GO! ------\n")

def follow(sensor : Sensor):
    if sensor.value.__class__ is float:
        print(round(sensor.value, 1))
    else:
        print(sensor.value)


"""
A0 = D14
A1 = D15
A2 = D16
A3 = D17
A4 = D18
A5 = D19
"""