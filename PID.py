import grovepi
import brickpi3
import sys
from enum import Enum
from time import sleep, time
from math import copysign, pi, sin, cos, radians
from MPU9250 import MPU9250
from IMUFilters import *
import numpy as np

# initialization
BP = brickpi3.BrickPi3()
IMU = MPU9250()

BPPort = {1:BP.PORT_1, 2:BP.PORT_2, 3:BP.PORT_3, 4:BP.PORT_4, 
                "A":BP.PORT_A,"B":BP.PORT_B, "C":BP.PORT_C, "D":BP.PORT_D}

# Constants
basePower = 60
reduce = basePower * 0.24
maxDistance = 200
maxPower = 90
minPower = 35
period = 0.02 # seconds
g = 9.81 # m/s^2
wheelRadius = 1.42875 # cm
ratio = 0.5
rotationDist = 2 * pi * wheelRadius * ratio + 0.33 # cm

robotWidth = 22 # cm
wallDistance = 9  # cm

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
        """Updates the sensor value and returns it"""
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

        #adjsut for sensor max range
        if self.mode == sensorType.ultraSonicNXT: 
                if self.value == 255: self.value = 123
        elif self.mode == sensorType.ultraSonic:
                if self.value >= 490: self.value = 135

        if self.biased: self.value = (self.value + self.bias) * self.multiplier
        return self.value
    
    def calibrateBias(self, samples = 100, delay = period, multi = 1.2):
        """Calibrates the bias of the sensor by taking a number of samples and averaging them."""
        self.biased = True
        recordings = []
        for i in range(samples):
            recordings.append(self.update())
            sleep(delay)
        self.bias = -np.mean(recordings) * multi
    
    @classmethod
    def updateAll(cls):
        """Updates all sensors"""
        cls.Inertial.update()
        for sensor in cls.allSensors:
            sensor.update()
        

class IMUWrap():
    
    def __init__(self) -> None:
        self.magBias   : np.ndarray = np.array([0, 0, 0], dtype="float64")
        self.magnetic     : np.ndarray = None
        self.magThreshold = 0
    
    def setThreshold(self, threshold):
        """Sets the threshold for the magnetNear function."""
        self.magThreshold = threshold

    def setBias(self):
        """Sets the bias for the magnetic reading"""
        biases = AvgCali(IMU, 100, 0.04)
        self.magBias = np.array(biases[-3:len(biases)])

    def update(self, verbose = False):
        """Updates the magnetic reading."""
        self.magnetic = np.array([float(x) for x in IMU.readMagnet().values()]) - self.magBias

        if verbose:
            print("Magnetic: ", self.magnetic)

    
    @property
    def magneticMagnitude(self) -> float:
        """Returns the magnitude of the magnetic reading."""
        return np.sqrt((self.magnetic**2).sum())

    def magnetNear(self, threshold = None) -> bool:
        """Returns true if the magnetic reading is above the threshold (so 1 tile away)"""
        if threshold == None: threshold = self.magThreshold
        return self.magneticMagnitude >= threshold
    
        

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
        """Updates the motor power, capping it at maxPower and minPower."""
        if abs(self.power) > maxPower:
            self.power = copysign(maxPower, self.power)
        elif abs(self.power) < minPower:
            self.power = copysign(minPower, self.power)
        BP.set_motor_power(self.pin, self.power - 2 * self.power * self.reverse)

    def stop(self):
        BP.set_motor_power(self.pin, 0)

    def getPosition(self):
        BP.get_motor_encoder(self.pin)


class DriveTrain():                                                                 

    def __init__(self, leftpin, rightpin) -> None:
        self.left  = Motor(leftpin)
        self.right = Motor(rightpin)
        self.reverse = False

    def setAllPowers(self, power):
        """Sets the power of both motors to the same value."""
        self.left.setPower(power)
        self.right.setPower(power)
        self.reverse = power < 0
    
    def reduceLeft(self, reduce):
        self.left.power -= reduce

    def reduceRight(self, reduce):
        self.right.power -= reduce

    def adjustPowers(self, adjustment):
        """Positive adjustment increases right"""
        self.left.power += adjustment
        self.right.power -= adjustment

    def setLeft(self, power):
        self.left.setPower(power)

    def setRight(self, power):
        self.right.setPower(power)

    def turn(self, direction : bool, power):
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


    def __init__(self, Kp, Ki, Kd, sensor, target = 0, margin = 1, derivativeMax = 30) -> None:
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

        self.maxOutput = maxPower - 18

    # --------------- update section ---------------
    def update(self, verbose = False, value = None):
        self.lastError = self.error
        if value is None:
            self.error = self.target - self.sensor.value
        else:
            self.error = self.target - value


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
        """Sets the output of the PID controller."""
        self.partialOut    = self.error * self.Kp
        self.integralOut   = self.totalError * self.Ki
        self.derivativeOut = self.Kd*(self.error - self.lastError)/period

        self.output = min([self.partialOut + self.integralOut + (self.derivativeOut * (abs(self.derivativeOut) <= self.derivativeMax)), maxPower])
        self.output = copysign(min([abs(self.output), self.maxOutput]), self.output)

    def setTarget(self, target):
        self.target = target

    def goToTarget(self, motorFunc):
        """Moves until the error is within the margin for X frames."""
        goodFrames = 0
        while goodFrames <= 2:
            Sensor.updateAll()
            self.update(verbose = False)
            motorFunc(self.output)
            time.sleep(period)
            goodFrames = goodFrames + 1 if abs(self.error) < self.margin else 0
        
        # motorFunc(0)

    def resetIntegral(self):
        self.integralOut = 0




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

    def __repr__(self) -> str:
        return self.name

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
    
    def __hash__(self) -> int:
        return hash(tuple(self.position))
    
    def __getitem__(self, index):
        if index.__class__ == Direction:
            return self.edges[index.value]
        return self.edges[index]

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

class Hazard():

    def __init__(self, position, intensity, parameter, name) -> None:
        self.position = np.array(position)
        self.intensity = intensity
        self.parameter = parameter
        self.name = name
    
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
        self.hazards = []

    def setup(self, startingPos, scale):
        """Sets up the map with the starting position and scale"""
        self.scale = scale

        for y in range(self.tiles.shape[0]):
            for x in range(self.tiles.shape[1]):
                self.tiles[y][x] = Tile((y, x))

        self.currentTile = self.tiles[tuple(startingPos)]
        self.currentTile.type = tileType.origin

    def setTileType(self, position : tuple, type : tileType):
        self.tiles[position].type = type

    def setTileEdge(self, position : tuple, direction : Direction, type : edgeType):
        self.tiles[position].setEdge(direction, type)
    
    def getGridPosition(self, position):
        """Returns the grid position of a given spacial position vector"""
        return (round(position[1] / (self.scale-0.5)), round(position[0] / (self.scale-0.5)))
        
    
    def updateCurrentTile(self):
        """Updates the current tile based on the navigator's position, 
        determines if the exit is found"""
        position = self.getGridPosition(self.navigator.position)
        try:
            if position[0] < 0 or position[1] < 0: raise IndexError
            self.currentTile = self.tiles[position]
            print("--- POSITION:", self.currentTile.position)
        except IndexError:
            self.currentTile = None
            raise outException
    
    def update(self):
        try:
            self.updateCurrentTile()
            if self.currentTile.type == tileType.unknown: self.currentTile.type = tileType.path
            self.updateEdges()
        except outException:
            self.out = True


    def updateEdges(self):
        """Updates the edges of the current tile based on the navigator's distance sensors"""
        for direction in Direction:
            print(direction.name, self.navigator.distances[direction])
            # set edge of current tile
            if self.navigator.distances[direction] == None: continue
            elif self.navigator.distances[direction] < self.scale: continue

            tileShift = 1
            tilePosition = self.currentTile.position.copy()
            edgeFound = False
            
            while (tileShift) * self.scale < self.navigator.distances[direction]:
                # print("TILE:", tilePosition)
                self.tiles[tuple(tilePosition)].setEdge(direction, edgeType.open)
                tilePosition += self.directionalAdditors[direction]
                try:
                    if tilePosition[0] < 0 or tilePosition[1] < 0: raise IndexError
                    self.tiles[tuple(tilePosition)].setEdge(Direction.flip(direction), edgeType.open)
                except IndexError:
                    if self.tiles[tuple(tilePosition - self.directionalAdditors[direction])].type != tileType.origin:
                        self.exitTile = self.tiles[tuple(tilePosition - self.directionalAdditors[direction])]
                        self.exitTile.type = tileType.exit
                        self.exitDirection = direction
                        print("FOUND EXIT AT", self.exitTile.position)
                    edgeFound = True
                    break
                tileShift += 1
                
            
            if not edgeFound:
                self.tiles[tuple(tilePosition)].setEdge(direction, edgeType.wall)
    

    def checkExplored(self, originalTile, originalDirection):
        """Checks the percentage of explored tiles from a given tile in a given direction"""
        totalTiles = [originalTile]
        stepTiles = [originalTile]
        nextStepTiles = [self.tiles[tuple(originalTile.position + self.directionalAdditors[originalDirection])]]
        totalTiles.append(nextStepTiles[0])

        while len(stepTiles) > 0:
            stepTiles = [x for x in nextStepTiles]
            nextStepTiles = []
            for tile in stepTiles:
                for checkDirection in Direction:
                    if tile.edges[checkDirection.value] == edgeType.open:
                        try:
                            newTile = self.tiles[tuple(tile.position + self.directionalAdditors[checkDirection])]
                            if newTile not in totalTiles:
                                totalTiles.append(newTile)
                                nextStepTiles.append(newTile)
                            
                        except IndexError:
                            pass
        
        totalTiles.remove(originalTile)
        numUnknowTiles = len([tile for tile in totalTiles if tile.type == tileType.unknown])
        unknownPercentage = 0 if len(totalTiles) == 0 else numUnknowTiles / len(totalTiles)

        return (numUnknowTiles, unknownPercentage)

    def getAdjacentTiles(self, tile : Tile):
        """Returns a dictionary of the adjacent tiles to a given tile"""
        adjacentTiles = {}
        for direction in Direction:
            if tile.edges[direction.value] == edgeType.open:
                adjacentTiles[direction] = self.tiles[tuple(tile.position + self.directionalAdditors[direction])]
        return adjacentTiles
    
    def addHeatHazard(self, direction : Direction, intensity):
        """Adds a heat hazard to the map in a given direction from the current tile"""
        self.tiles[tuple(self.currentTile.position + self.directionalAdditors[direction])].type = tileType.heatHazard
        print("ADDED HEAT:", self.currentTile.position, self.currentTile.position + self.directionalAdditors[direction])
        position = self.currentTile.position + self.directionalAdditors[direction]
        position = (position[1] * self.scale, position[0] * self.scale)
        self.hazards.append(Hazard(position, intensity, "Radiated Power (W)", "Heat Hazard"))
        
    
    def addMagnetHazard(self, direction : Direction, intensity):
        """Adds a magnet hazard to the map in a given direction from the current tile"""
        self.tiles[tuple(self.currentTile.position + self.directionalAdditors[direction])].type = tileType.magnetHazard
        print("ADDED MAGNET:", self.currentTile.position, self.currentTile.position + self.directionalAdditors[direction])
        position = self.currentTile.position + self.directionalAdditors[direction]
        position = (position[1] * self.scale, position[0] * self.scale)
        self.hazards.append(Hazard(position, intensity, "Magnetic Field Strength (uT)", "Magnet Hazard"))

    def printMapEdges(self):
        for tile in self.tiles.flat:
            print(tile.position, tile.edges, tile.type.name)
    

    def makeMap(self):
        """Makes a map from the current tiles and saves it to a csv file"""
        tileTypes = np.array([[tile.type.value for tile in row] for row in self.tiles], dtype="int16")

        #get origin tile
        originTile : Tile = None
        for tile in self.tiles.flat:
            if tile.type == tileType.origin:
                originTile = tile
                break

        mapNum = input('Which map number? ')

        with open("team87_map.csv", "w") as file:
            file.write("Team: 87\n")
            file.write(f"Map: {mapNum}\n")
            file.write(f"Unit length: {self.scale}\n")
            file.write(f"Origin: {originTile.position[0]}, {originTile.position[1]}\n")
            file.write("Notes: \n")
            for row in range(tileTypes.shape[0] - 1, -1, -1):
                for col in range(tileTypes.shape[1]):
                    file.write(str(tileTypes[row][col]))
                    if col < tileTypes.shape[1] - 1: file.write(",")
                file.write("\n")
        
        with open("team87_hazards.csv", "w") as file:
            file.write("Team: 87\n")
            file.write(f"Map: {mapNum}\n")
            file.write("Notes: \n\n")
            file.write("Resource Type,Parameter of Interest,Parameter,Resource X Coordinate, Resource Y Coordinate\n")
            for hazard in self.hazards:
                hazard : Hazard
                file.write(f"{hazard.name},{hazard.parameter},{hazard.intensity},{hazard.position[0]} cm,{hazard.position[1]} cm\n")
            



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
        # self.magDirection = Direction.flip(self.magDirection)

        if self.magneticSensor.magnetNear():
            self.map.addMagnetHazard(self.magDirection, self.magneticSensor.magneticMagnitude)

    def updateIRReading(self):
        self.foundIR = self.IRSensor.value > 0.0
        if self.foundIR:
            self.map.addHeatHazard(self.direction, self.IRSensor.value - self.IRSensor.bias)

    def flushMap(self):
        self.map.update()

    def getInstruction(self) -> NavigationOption:
        """Returns the next instruction for the robot to follow
        considering the current state of the map and the sensors"""
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
                preferences[tile] = preference

            finalDirection = adjacentTiles.keys().__iter__().__next__()
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
            print("Exception:", e)
            finalDirection = self.map.exitDirection
        


        if finalDirection == self.direction:
            return NavigationOption.drive
        elif finalDirection == Direction.flip(self.direction):
            return NavigationOption.turn180
        elif finalDirection == Direction((self.direction.value + 1) % 4):
            return NavigationOption.turnClock90
        elif finalDirection == Direction((self.direction.value + 3) % 4):
            return NavigationOption.turnCounterClock90

        print("Didnt find instruction")
        print("final dir:", finalDirection)
    
    @property
    def position(self):
        return self.positionTracker.position
    




def delay():
    input("Press Enter when ready")
    print("------  GO! ------\n")


"""
A0 = D14
A1 = D15
A2 = D16
A3 = D17
A4 = D18
A5 = D19
"""