from setup import *

# drive.setAllPowers(60)
# drive.update()

target = 10


while True:
    # Sensor.updateAll()
    driveStraight(basePower)

    Position.update()

    print(f"{Position}   {PositionTracker.heading:3}\n")
    if (Position.y > target): break

    # print("")

    sleep(period)


raise KeyboardInterrupt