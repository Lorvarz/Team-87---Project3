import grovepi
import brickpi3
import sys
from enum import Enum
from time import sleep, time
from math import copysign, pi, sin, cos, radians
from MPU9250 import MPU9250
from IMUFilters import *
import numpy as np
from Mapping import *

BP = brickpi3.BrickPi3()
IMU = MPU9250()

BPPort = {1:BP.PORT_1, 2:BP.PORT_2, 3:BP.PORT_3, 4:BP.PORT_4, 
                "A":BP.PORT_A,"B":BP.PORT_B, "C":BP.PORT_C, "D":BP.PORT_D}

basePower = 65
reduce = basePower * 0.15
maxDistance = 200
maxPower = 90
minPower = 25
period = 0.02 # seconds
g = 9.81 # m/s^2
wheelRadius = 1.42875 # cm
ratio = 0.5
rotationDist = 2 * pi * wheelRadius * ratio + 0.83 # cm

robotWidth = 22 # cm
wallDistance = (40 - 22) / 2  # cm

# table: 2.6  my table: 3.0
magicGyroConstant = 3.1



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
    ultraSonicNXT = BP.SENSOR_TYPE.NXT_ULTRASONIC
    IMU = 7
    motor = 8
    inertialRotation = 9
    inertialHeading = 10
    
    

class Sensor():

    allSensors = []
    Inertial = None

    def __init__(self, type, mode, pin, biased = False, bias = 0, multiplier = 1) -> None:
        self.type = type
        self.mode = mode
        self.pin = pin
        self.value = 0
        self.biased = biased
        self.bias = bias
        self.multiplier = multiplier
        self.allSensors.append(self)
        if(type == sensorType.grove): grovepi.pinMode(pin, "INPUT")
        elif (type == sensorType.brick and mode != sensorType.motor): BP.set_sensor_type(BPPort[pin], self.mode)
    
    def update(self) -> float:
        if(self.type == sensorType.grove): 
            if(self.mode == sensorType.analog):       self.value = grovepi.analogRead(self.pin)
            elif(self.mode == sensorType.digital):    self.value = grovepi.digitalRead(self.pin)
            elif(self.mode == sensorType.ultraSonic): self.value = grovepi.ultrasonicRead(self.pin)
            elif(self.mode == sensorType.hall): self.value = not grovepi.digitalRead(self.pin)
        elif (self.type == sensorType.brick):
            if(self.mode == sensorType.motor): self.value = BP.get_motor_encoder(BPPort[self.pin])
            elif(self.mode == sensorType.gyro):  self.value = BP.get_sensor(BPPort[self.pin])[0]
            else:                              self.value = BP.get_sensor(BPPort[self.pin])
        elif (self.type == sensorType.IMU):
            if (self.mode == sensorType.inertialRotation): self.value = self.Inertial.rotation
            elif (self.mode == sensorType.inertialHeading): self.value = self.Inertial.heading


        if self.biased: self.value = (self.value + self.bias) * self.multiplier
        return self.value
    
    @classmethod
    def updateAll(cls):
        cls.Inertial.update()
        for sensor in cls.allSensors:
            sensor.update()
        

class IMUWrap():
    
    driveRef = None

    def __init__(self) -> None:
        self.accelBias : np.ndarray = np.array([0, 0, 0], dtype="float64")
        self.gyroBias  : np.ndarray = np.array([0, 0, 0], dtype="float64")
        self.magBias   : np.ndarray = np.array([0, 0, 0], dtype="float64")

        self.accelNull : np.ndarray = np.array([[0, 0], [0, 0], [0, 0]])
        self.gyroNull  : np.ndarray = np.array([[0, 0], [0, 0], [0, 0]])
        self.magNull   : np.ndarray = np.array([[0, 0], [0, 0], [0, 0]])

        self.acceleration : np.ndarray = None
        self.gyro         : np.ndarray = None
        self.magnetic     : np.ndarray = None

        self.magThreshold = 0

        self.velocity = np.array([0, 0, 0], dtype="float64") # meters/sec
        self.position = np.array([0, 0, 0], dtype="float64") # meters

        self.rotation = 0 # degrees
        self.heading = 0 # degrees (0 - 360)

    def setBias(self):
        biases = AvgCali(IMU, 100, 0.04)
        # splits biases into 3 sublists and assigns them accordingly
        self.accelBias, self.gyroBias, self.magBias = [np.array(biases[x: x+3]) for x in range(0, len(biases), 3)]

    def getNullBand(self, maxTime = period * 250):
        multi = 1.1
        elapsed = 0
        index = 0
        accelData = np.ndarray((3, int(maxTime/period + 1)))

        while elapsed < maxTime:
            Sensor.Inertial.update(kinematics = False)
            for i in range(3):
                accelData[i][index] = Sensor.Inertial.acceleration[i]

            elapsed += period
            index += 1

            time.sleep(period)

        self.accelNull = np.ndarray((3, 2))
        # self.gyroNull = np.ndarray((3, 2))
        # self.magNull = np.ndarray((3, 2))

        for i in range(3):
            self.accelNull[i][0] = accelData[i].min() * multi
            self.accelNull[i][1] = accelData[i].max() * multi
            # self.gyroNull[i][0] = accelData[i].min() * multi
            # self.gyroNull[i][1] = accelData[i].max() * multi
            # self.magNull[i][0] = accelData[i].min() * multi
            # self.magNull[i][1] = accelData[i].max() * multi
    


    def update(self, kinematics = False, verbose = False, selection = None):
        self.acceleration = np.array([float(x) for x in IMU.readAccel().values()])  - self.accelBias
        self.gyro         = np.array([float(x) for x in IMU.readGyro().values()])   - self.gyroBias
        self.magnetic     = np.array([float(x) for x in IMU.readMagnet().values()]) - self.magBias

        # for i in range(3):
        #     if self.acceleration[i] > self.accelNull[i][0] and self.acceleration[i] < self.accelNull[i][1]:
        #         self.acceleration[i] = 0

        # if kinematics: self.kinematics()

        self.updateRotation()

        if verbose:
            if selection == 1:
                print("Acceleration: ", self.acceleration)
            elif selection == 2:
                print("Gyro: ", self.gyro)
            elif selection == 3:
                print("Magnetic: ", self.magnetic)

    
    @property
    def magneticMagnitude(self) -> float:
        return np.sqrt((self.magnetic**2).sum())
    
    @property
    def velocityCm(self) -> np.ndarray: # returns the velocity in cm
        return self.velocity * 100
    
    @property
    def positionCm(self) -> np.ndarray: # returns the position in cm
        return self.position * 100

    def onBeacon(self) -> bool:
        return self.magneticMagnitude > self.magThreshold

    def kinematics(self):
        acceleration   = self.acceleration * g
        self.velocity += acceleration * period
        self.position += self.velocity * period

    def updateRotation(self):
        self.rotation += self.gyro[2] * period * magicGyroConstant
        self.heading = copysign(abs(self.rotation) % 360, self.rotation) 
        if self.heading < 0: self.heading += 360
        



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
        if abs(self.power) > maxPower:
            self.power = copysign(maxPower, self.power)
        elif abs(self.power) < minPower:
            self.power = copysign(minPower, self.power)
        BP.set_motor_power(self.pin, self.power - 2 * self.power * self.reverse)

    def stop(self):
        BP.set_motor_power(self.pin, 0)

    def getPosition(self):
        BP.get_motor_encoder(BPPort[self.pin])


class DriveTrain():                                                                 

    def __init__(self, leftpin, rightpin) -> None:
        self.left  = Motor(leftpin)
        self.right = Motor(rightpin)
        self.reverse = False

    def setAllPowers(self, power):
        self.left.setPower(power)
        self.right.setPower(power)
        self.reverse = power < 0
    
    def reduceLeft(self, reduce):
        # self.left.power -= copysign(reduce, self.left.power)
        self.left.power -= reduce

    def reduceRight(self, reduce):
        # self.right.power -= copysign(reduce, self.right.power)
        self.right.power -= reduce

    def setLeft(self, power):
        self.left.setPower(power)

    def setRight(self, power):
        self.right.setPower(power)

    def turn(self, direction : bool, power):
        # :param direction: True = Right, False = left
        self.setRight(power - 2 * power * direction)
        self.setLeft(power - 2 * power * (not direction))

    def stop(self):
        self.left.stop()
        self.right.stop()
    
    def update(self):
        self.left.update()
        self.right.update()

# -------------------- Position Tracker --------------------
class PositionTracker():

    gyro = None
    heading = 0

    def __init__(self, left, right) -> None:
        self.encoders = [left, right]
        self.lastEncoder = [0, 0]
        
        self.position = np.array([0.0, 0.0], dtype="float64") # x, y (cm)

    def __str__(self) -> str:
        return f"X: {self.position[0]:6.2f}, Y: {self.position[1]:6.2f}"
    
    @classmethod
    def updateHeading(cls):
        cls.heading = copysign(abs(cls.gyro.value) % 360, cls.gyro.value)
        if cls.heading < 0: cls.heading += 360


    def update(self):
        self.updateHeading()
        avgReading = (self.encoders[0].value - self.lastEncoder[0] + self.encoders[1].value - self.lastEncoder[1]) / 2
        self.position[0] += avgReading / (360) * rotationDist * sin(radians(self.heading))
        self.position[1] += avgReading / (360) * rotationDist * cos(radians(self.heading))

        self.setLast()

    def setLast(self, update = False):
        if update:
            for encoder in self.encoders: encoder.update()
        self.lastEncoder[0] = self.encoders[0].value
        self.lastEncoder[1] = self.encoders[1].value

    @property
    def x(self) -> float:
        return self.position[0]
    
    @property
    def y(self) -> float:
        return self.position[1]

# -------------------- PID --------------------

class PIDController():


    def __init__(self, Kp, Ki, Kd, sensor, target = 0, margin = 1, derivativeMax = 30, axis = None) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.sensor = sensor
        self.target = target

        self.error      = 0
        self.lastError  = 0
        self.totalError = 0

        self.margin = 1

        self.derivativeMax = derivativeMax

        self.partialOut    = 0
        self.integralOut   = 0
        self.derivativeOut = 0
        self.output        = 0

        self.axis = axis
        self.maxOutput = maxPower - 18

    # --------------- update section ---------------
    def update(self, verbose = False):
        self.lastError = self.error
        if self.axis != None:
            self.error = self.target - self.sensor.value[self.axis]
        else:
            self.error = self.target - self.sensor.value


        self.totalError += self.error * period * (abs(self.integralOut) <= maxPower)
        self.setOutput()
        if verbose:
            self.printInfo()
    
    def printInfo(self):
        print(f"Reading:           {self.sensor.value:.2f}")
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
        self.output = copysign(min([abs(self.output), self.maxOutput]), self.output)

    def setTarget(self, target):
        self.target = target


    def goToTarget(self, motorFunc):
        goodFrames = 0
        while goodFrames <= 2:
            Sensor.updateAll()
            self.update(verbose = True)
            motorFunc(self.output)
            time.sleep(period)
            goodFrames = goodFrames + 1 if abs(self.error) < self.margin else 0
        
        # motorFunc(0)




class NavigationOption(Enum):
    Drive = 0
    Turn = 1
    check = 2

class Direction(Enum):
    up = 0
    right = 1
    down = 2
    left = 3

class Instruction():

    def __init__(self, option : NavigationOption, direction : Direction) -> None:
        self.command = option
        self.direction = direction


class Tile():
    pass



class Navigator():

    def __init__(self, initialDirection, positionTracker, distanceSensors, IR, magnetic, map) -> None:
        self.direction : Direction = initialDirection
        self.positionTracker = positionTracker
        self.distanceSensors : dict[Direction , Sensor] = distanceSensors
        self.distances : dict[Direction , float] = {Direction.up : None, Direction.right : None, 
                                                    Direction.down : None, Direction.left : None}
        self.IRSensor = IR
        self.magneticSensor = magnetic
        self.map = map


    
    def update(self):
        self.updatePosition()
        self.updateDirection()
        self.updateDistances()
        self.updateOtherSensors()
        self.flushMap()


    def updatePosition(self):
        self.positionTracker.update()

    
    def updateDirection(self):
        self.direction = Direction(self.positionTracker.heading // 90)


    def updateDistances(self):
        for sensorDirection, sensor in self.distanceSensors.items():
            sensor.update()
            finalDir = (sensorDirection.value + self.direction.value) % 4
            self.distances[Direction(finalDir)] = sensor.value
        self.distances[Direction((self.direction.value + 2) % 4)] = None
    
    def updateOtherSensors(self):
        self.IRSensor.update()
        self.magneticSensor.update()


    






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