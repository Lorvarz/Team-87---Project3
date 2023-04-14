from setup import *

scale = areaMap.scale

while True:
    Position.update()
    command = input("Enter the command: ").split(" ")

    option = command[0]
    distance = int(command[1])

    if option == "drive":
        driveDistance(distance * scale)
    elif option == "turn":
        turnRelative(distance)
        Position.setLast(True)