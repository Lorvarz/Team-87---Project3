import numpy as np
from enum import Enum

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
            tilePosition = self.currentTile.position.copy()
            foundExit = False
            
            while (tileShift) * self.scale < self.navigator.distances[direction]:
                print("tile", tuple(tilePosition))
                print("current", self.currentTile.position)
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
        stepTiles = [originalTile]
        nextStepTiles = [self.tiles[tuple(originalTile.position + self.directionalAdditors[originalDirection])]]
        totalTiles.append(nextStepTiles[0])

        print("nextStepTiles", nextStepTiles[0].position)

        while len(stepTiles) > 0:
            stepTiles = [x for x in nextStepTiles]
            nextStepTiles = []
            for tile in stepTiles:
                for checkDirection in Direction:
                    if tile.edges[checkDirection.value] == edgeType.open:
                        try:
                            newTile = self.tiles[tuple(tile.position + self.directionalAdditors[checkDirection])]
                            print("newTile", newTile.position)
                            print("totalTiles", [x.position for x in totalTiles])

                            # check if newTile is in totalTiles
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



myMap = Map((4,4))
myMap.setup((0,0), 10)

nav = Navigator(Direction.up, None, None, None, None, myMap)

nav.distances[Direction.up] = 7
nav.distances[Direction.right] = 35
nav.distances[Direction.down] = None
nav.distances[Direction.left] = None

myMap.updateEdges()

for tile in myMap.tiles.flat:
    print(tile.position, tile.edges, tile.type.name)

tile = myMap.tiles[(0,2)]
print(tile.position, tile.type)
print(myMap.checkExplored(myMap.tiles[(0,0)], Direction.right))