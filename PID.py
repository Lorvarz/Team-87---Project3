import grovepi
import brickpi3
import sys
from enum import Enum
from time import sleep, time
from math import copysign, pi, sin, cos, radians
from MPU9250 import MPU9250
from IMUFilters import *
import numpy as np

BP = brickpi3.BrickPi3()
# IMU = MPU9250()

BPPort = {1:BP.PORT_1, 2:BP.PORT_2, 3:BP.PORT_3, 4:BP.PORT_4, 
                "A":BP.PORT_A,"B":BP.PORT_B, "C":BP.PORT_C, "D":BP.PORT_D}

basePower = 60
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
    IMUmag = 8
    motor = 9
    
    

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
            if (self.mode == sensorType.IMUmag): self.value = self.Inertial.magnetic


        if self.biased: self.value = (self.value + self.bias) * self.multiplier
        return self.value
    
    def calibrateBias(self, samples = 100, delay = period, multi = 1.2):
        self.biased = True
        recordings = []
        for i in range(samples):
            recordings.append(self.update())
            sleep(delay)
        self.bias = -np.mean(recordings) * multi
    
    @classmethod
    def updateAll(cls):
        cls.Inertial.update()
        for sensor in cls.allSensors:
            sensor.update()
        

class IMUWrap():
    
    def __init__(self) -> None:
        self.magBias   : np.ndarray = np.array([0, 0, 0], dtype="float64")

        self.magNull   : np.ndarray = np.array([[0, 0], [0, 0], [0, 0]])

        self.magnetic     : np.ndarray = None

        self.magThreshold = 0

    def setBias(self):
        biases = AvgCali(IMU, 100, 0.04)
        self.magBias = np.array(biases[-3:len(biases)])

    def update(self, verbose = False):
        self.magnetic = np.array([float(x) for x in IMU.readMagnet().values()]) - self.magBias

        if verbose:
            print("Magnetic: ", self.magnetic)

    
    @property
    def magneticMagnitude(self) -> float:
        return np.sqrt((self.magnetic**2).sum())
    
        

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




# -------------------- Navigation and Mapping --------------------


class NavigationOption(Enum):
    drive = 0    # drive forward
    turnClock90 = 1   # turn 90 degrees Clockwise
    turnCounterClock90 = 2   # turn left 90 degrees CounterClockwise
    turn180 = 3  # turn 180 degrees

class Direction(Enum):
    up = 0
    right = 1
    down = 2
    left = 3

    @classmethod
    def flip(cls, direction):
        return cls((direction.value + 2) % 4)

class tileType(Enum):
    unknown = 0
    path = 1
    origin = 5
    heatHazard = 2
    magnetHazard = 3
    exit = 4

class edgeType(Enum):
    open = 0
    wall = 1
    unknown = 2

# -------------------- Position Tracker --------------------
class PositionTracker():

    gyro = None
    heading = 0

    def __init__(self, left, right) -> None:
        self.encoders : list(Sensor) = [left, right]
        self.lastEncoder = [0, 0]
        self.direction = None
        
        self.position = np.array([0.0, 0.0], dtype="float64") # x, y (cm)

    def __str__(self) -> str:
        return f"X: {self.position[0]:6.2f}, Y: {self.position[1]:6.2f}"
    
    @classmethod
    def updateHeading(cls):
        cls.gyro.update()
        cls.heading = copysign(abs(cls.gyro.value) % 360, cls.gyro.value)
        if cls.heading < 0: cls.heading += 360

    @property
    def rotation(self):
        return self.gyro.value
    
    def updateEncoders(self):
        for encoder in self.encoders: encoder.update()

    def update(self):
        self.updateHeading()
        self.updateDirection()
        self.updateEncoders()
        avgReading = (self.encoders[0].value - self.lastEncoder[0] + self.encoders[1].value - self.lastEncoder[1]) / 2
        self.position[0] += avgReading / (360) * rotationDist * sin(radians(self.heading))
        self.position[1] += avgReading / (360) * rotationDist * cos(radians(self.heading))

        self.setLast()

    def setLast(self, update = False):
        if update:
            for encoder in self.encoders: encoder.update()
        self.lastEncoder[0] = self.encoders[0].value
        self.lastEncoder[1] = self.encoders[1].value

    def updateDirection(self):
        self.direction = Direction(((self.heading + 45) // 90 ) % 4)

    @property
    def x(self) -> float:
        return self.position[0]
    
    @x.setter
    def x(self, value):
        self.position[0] = value
    
    @property
    def y(self) -> float:
        return self.position[1]
    
    @y.setter
    def y(self, value):
        self.position[1] = value

class Tile():
    
    def __init__(self, position) -> None:
        self.position = np.array(position)
        self.type = tileType.unknown
        self.edges = [edgeType.unknown, edgeType.unknown, edgeType.unknown, edgeType.unknown]
    
    def __eq__(self, __value: object) -> bool:
        return self.position[0] == __value.position[0] and self.position[1] == __value.position[1]

    @property
    def up(self):
        return self.edges[0]
    
    @up.setter
    def up(self, value : edgeType):
        self.edges[0] = value

    @property
    def right(self):
        return self.edges[1]
    
    @right.setter
    def right(self, value : edgeType):
        self.edges[1] = value

    @property
    def down(self):
        return self.edges[2]
    
    @down.setter
    def down(self, value : edgeType):
        self.edges[2] = value

    @property
    def left(self):
        return self.edges[3]
    
    @left.setter
    def left(self, value : edgeType):
        self.edges[3] = value

    def setEdge(self, direction : Direction, value : edgeType):
        self.edges[direction.value] = value
        return self

class outException(Exception):
    pass

class Map():
    # all position vectors are in the form (y, x)

    directionalAdditors = { Direction.up : np.array((1, 0)), 
                            Direction.right : np.array((0, 1)), 
                            Direction.down : np.array((-1, 0)), 
                            Direction.left : np.array((0, -1))}


    def __init__(self, shape) -> None:
        self.tiles = np.ndarray(shape, dtype = Tile)
        self.currentTile = None
        self.navigator : Navigator = None
        self.scale = None
        self.exit = None
        self.exitDirection = None
        self.out = False

    def setup(self, startingPos, scale):
        self.scale = scale

        for y in range(self.tiles.shape[0]):
            for x in range(self.tiles.shape[1]):
                self.tiles[y][x] = Tile((y, x))

        self.currentTile = self.tiles[tuple(startingPos[::-1])]
        self.currentTile.type = tileType.origin

    def setTileType(self, position : tuple, type : tileType):
        self.tiles[position].type = type

    def setTileEdge(self, position : tuple, direction : Direction, type : edgeType):
        self.tiles[position].setEdge(direction, type)
    
    def getGridPosition(self, position):
        return (round(position[1] / self.scale), round(position[0] / self.scale))
    
    def updateCurrentTile(self):
        try:
            self.currentTile = self.tiles[self.getGridPosition(self.navigator.position)[::-1]]
        except IndexError:
            self.currentTile = None
            raise outException
    
    def update(self):
        try:
            self.updateCurrentTile()
            self.currentTile.Type = tileType.path
            self.updateEdges()
        except outException:
            self.out = True


    def updateEdges(self):
        for direction in Direction:
            print(direction.name, self.navigator.distances[direction])
            # set edge of current tile
            if self.navigator.distances[direction] == None: continue

            tileShift = 1
            tilePosition = self.currentTile.position[:]
            foundExit = False
            
            while (tileShift) * self.scale < self.navigator.distances[direction]:
                print(tuple(tilePosition))
                self.tiles[tuple(tilePosition)].setEdge(direction, edgeType.open)
                tilePosition += self.directionalAdditors[direction]
                try:
                    self.tiles[tuple(tilePosition)].setEdge(Direction.flip(direction), edgeType.open)
                except IndexError:
                    if self.tiles[tuple(tilePosition - self.directionalAdditors[direction])].type != tileType.origin:
                        self.exitTile = self.tiles[tuple(tilePosition - self.directionalAdditors[direction])].type = tileType.exit
                        self.exitDirection = direction
                        foundExit = True
                tileShift += 1
            
            if not foundExit:
                self.tiles[tuple(tilePosition)].setEdge(direction, edgeType.wall)
    

    def checkExplored(self, originalTile, originalDirection):
        totalTiles = [originalTile]
        stepTiles = [1]
        nextStepTiles = [self.tiles[tuple(originalTile.position + self.directionalAdditors[originalDirection])]]
        totalTiles.append(stepTiles[0])

        while len(stepTiles) > 0:
            stepTiles = nextStepTiles[:]
            nextStepTiles = []
            for tile in stepTiles:
                for checkDirection in Direction:
                    if tile.edges[checkDirection.value] == edgeType.open:
                        newTile = self.tiles[tuple(tile.position + self.directionalAdditors[checkDirection])]
                        if newTile not in totalTiles:
                            nextStepTiles.append(newTile)
                            totalTiles.append(newTile)
        
        totalTiles.remove(originalTile)
        numUnknowTiles = len([tile for tile in totalTiles if tile.type == tileType.unknown])
        unknownPercentage = numUnknowTiles / len(totalTiles)

        return (numUnknowTiles, unknownPercentage)

    def getAdjacentTiles(self, tile : Tile):
        adjacentTiles = {}
        for direction in Direction:
            if tile.edges[direction.value] == edgeType.open:
                adjacentTiles[direction] = self.tiles[tuple(tile.position + self.directionalAdditors[direction])]
        return adjacentTiles
    

    def makeMap(self):
        file = open("Map.txt", "w")

        with file:
            file.write("Team: 87\n")
            file.write("Map: 0")
            file.write(f"Scale: {self.scale}")
            for row in self.tiles[::-1]:
                for tile in row:
                    tile : Tile
                    file.write(f"{tile.type.value} \t")
                tile.write("\n")



class Navigator():

    tilesRemembered = 5
    
    tilePreference = {tileType.unknown : 3,
                      tileType.path : 2,
                      tileType.heatHazard : 0,
                      tileType.magnetHazard : 0,
                      tileType.exit : 5,
                      tileType.origin : 1}

    def __init__(self, initialDirection, positionTracker, distanceSensors, IR, magnetic, map) -> None:
        self.direction : Direction = initialDirection
        self.positionTracker : PositionTracker = positionTracker
        self.distanceSensors : dict[Direction , Sensor] = distanceSensors
        self.distances : dict[Direction , float] = {Direction.up : None, Direction.right : None, 
                                                    Direction.down : None, Direction.left : None}
        
        self.IRSensor : Sensor = IR
        self.foundIR = False

        self.magneticSensor : IMUWrap = magnetic
        self.magDirection = None

        self.map : Map = map
        map.navigator = self

        self.lastTiles = [Tile((-1,-1)) for i in range(self.tilesRemembered)]
    
    def update(self):
        self.lastTiles = self.lastTiles[1:] + [self.map.currentTile]
        self.updatePosition()
        self.updateDirection()
        self.updateDistances()
        self.updateOtherSensors()
        self.updateMagDirection()
        self.updateIRReading()
        self.flushMap()
        return self.getInstruction()

    def updatePosition(self):
        self.positionTracker.update()
    
    def updateDirection(self):
        self.direction = Direction(((self.positionTracker.heading + 45) // 90 ) % 4)

    def updateDistances(self):
        for sensorDirection, sensor in self.distanceSensors.items():
            sensor.update()
            finalDir = (sensorDirection.value + self.direction.value) % 4
            self.distances[Direction(finalDir)] = sensor.value
        self.distances[Direction.flip(self.direction)] = None
    
    def updateOtherSensors(self):
        self.IRSensor.update()
        self.magneticSensor.update()

    def updateMagDirection(self):
        x, y = self.magneticSensor.magnetic[0:2]
        if abs(x) > abs(y) and x > 0:
            greaterDirection = Direction.right
        elif abs(x) > abs(y) and x < 0:
            greaterDirection = Direction.left
        elif abs(y) > abs(x) and y > 0:
            greaterDirection = Direction.up
        elif abs(y) > abs(x) and y < 0:
            greaterDirection = Direction.down
        else:
            greaterDirection = Direction.up

        self.magDirection = Direction((greaterDirection.value + self.direction.value) % 4)

    def updateIRReading(self):
        self.foundIR = self.IRSensor.value > 0.0

    def flushMap(self):
        self.map.update()

    def getInstruction(self) -> NavigationOption:
        try:
            if self.map.exitDirection is not None: raise Exception("Exit Found")
            explorationValues = {}
            for direction in Direction:
                if self.map.currentTile.edges[direction.value] == edgeType.open:
                    explorationValues[direction] = self.map.checkExplored(self.map.currentTile, direction)

            adjacentTiles = self.map.getAdjacentTiles(self.map.currentTile)
            preferences = {}

            for tile in adjacentTiles.values():
                preference = self.tilePreference[tile.type]
                if tile in self.lastTiles: preference -= 1
                preferences[tile] = self.tilePreference[tile.type]

            finalDirection = adjacentTiles.keys()[0]
            for direction, tile in adjacentTiles.items():
                if preferences[tile] > preferences[adjacentTiles[finalDirection]]:
                    finalDirection = direction
                elif preferences[tile] == preferences[adjacentTiles[finalDirection]]:
                    if explorationValues[direction][0] > explorationValues[finalDirection][0]:
                        finalDirection = direction
                    elif explorationValues[direction][0] == explorationValues[finalDirection][0]:
                        if explorationValues[direction][1] > explorationValues[finalDirection][1]:
                            finalDirection = direction
                        elif explorationValues[direction][1] == explorationValues[finalDirection][1]:
                            if direction != self.direction:
                                finalDirection = direction
        except Exception as e:
            print(str(e))
            finalDirection = self.map.exitDirection
        

        if finalDirection == self.direction:
            return NavigationOption.drive
        elif finalDirection == Direction.flip(self.direction):
            return NavigationOption.turn180
        elif finalDirection == Direction((self.direction.value + 1) % 4):
            return NavigationOption.turnClock90
        elif finalDirection == Direction((self.direction.value + 3) % 4):
            return NavigationOption.turnCounterClock90
    
    @property
    def position(self):
        return self.positionTracker.position
    







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