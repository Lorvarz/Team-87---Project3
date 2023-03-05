from setup import *

drive.setAllPowers(10)
drive.update()


while True:
    Sensor.updateAll()
    Position.update()

    print(Position)
    follow(leftMotor)
    print(f"Heading: {Sensor.Inertial.heading}")

    print("")

    sleep(period)


raise KeyboardInterrupt