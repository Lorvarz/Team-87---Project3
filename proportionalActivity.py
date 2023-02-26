from PID import *

thisMotor = Motor("A")
motorSensor = Sensor(sensorType.brick, sensorType.motor, "A")

controller = PIDController(0.2, 0, 0, motorSensor)

try:
    while True:
        controller.update()

        thisMotor.setPower(controller.output)
        thisMotor.update()
        sleep(period)

except :
    thisMotor.setPower(0)
    thisMotor.update()
    BP.reset_all()
    