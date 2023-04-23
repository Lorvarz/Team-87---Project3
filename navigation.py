from setup import *

while not areaMap.out:
    print(Position.y, " ", Position.x)
    instruction = centralNav.update()
    print(instruction.name)

    if instruction == NavigationOption.drive:
        driveDistance(areaMap.scale)
    elif instruction == NavigationOption.turnClock90:
        angleTarget = (Position.rotation + 45) // 90 * 90 + 90
        turnRelative(angleTarget - Position.rotation)
        Position.setLast(True)
    elif instruction == NavigationOption.turnCounterClock90:
        angleTarget = (Position.rotation + 45) // 90 * 90 - 90
        turnRelative(angleTarget - Position.rotation)
        Position.setLast(True)
    elif instruction == NavigationOption.turn180:
        angleTarget = (Position.rotation + 45) // 90 * 90 + 180
        turnRelative(angleTarget - Position.rotation)
        Position.setLast(True)
    
    sleep(period * 2)


# TODO: Drop cargo and dance
driveForward(40)
sleep(0.7)
drive.stop()

raise KeyboardInterrupt