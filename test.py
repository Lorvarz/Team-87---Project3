from enum import Enum
import numpy as np

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
    
test = Tile([0, 0])
test.up = edgeType.wall
test.right = edgeType.wall
test.down = edgeType.open

print(test[Direction.up])