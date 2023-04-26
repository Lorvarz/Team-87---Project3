from setup import *

while not areaMap.out:
    print(Position.y, " ", Position.x)
    instruction = centralNav.update()
    print(instruction.name)

    # determine instruction
    if instruction == NavigationOption.drive:
        select = 1 if Position.direction.value % 2 == 0 else 0
        target = (Position.position[select] + (areaMap.scale / 2)) // areaMap.scale * areaMap.scale + areaMap.scale
        driveDistance(target - Position.position[select])

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
    
    PIDGyro.resetIntegral()
    PIDStraight.resetIntegral()
    
    sleep(period * 2)

# drop cargo
driveForward(40)
sleep(0.7)
drive.stop()
dropCargo()
sleep(0.8)

# turn to dance
turnRelative(20)
turnRelative(-40)
turnRelative(20)
turnRelative(-40)

sleep(1)

raise KeyboardInterrupt