from setup import *

while not areaMap.out:
    instruction = centralNav.update()

    if instruction == NavigationOption.drive:
        driveDistance(areaMap.scale)
    elif instruction == NavigationOption.turnClock90:
        turnRelative(90)
        Position.setLast(True)
    elif instruction == NavigationOption.turnCounterClock90:
        turnRelative(-90)
        Position.setLast(True)
    elif instruction == NavigationOption.turn180:
        turnRelative(180)
        Position.setLast(True)


# TODO: Drop cargo and dance
driveForward(40)
sleep(0.7)
drive.stop()

raise KeyboardInterrupt