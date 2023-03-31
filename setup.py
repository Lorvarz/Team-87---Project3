from PID import *

# --------------------- Sensors and PID setup --------------------------

# --* Sensors *--
Sensor.Inertial = IMUWrap()
Left  = Sensor(sensorType.grove, sensorType.ultraSonic, 2)
Right = Sensor(sensorType.grove, sensorType.ultraSonic, 3)
Front = Sensor(sensorType.brick, sensorType.ultraSonicNXT, 1)

IR = Sensor(sensorType.grove, sensorType.analog, 2)
IR2 = Sensor(sensorType.grove, sensorType.analog, 1)

LegoGyro = Sensor(sensorType.brick, sensorType.gyro, 4)

PIDGyro     = PIDController(1.3, 0.005, 0.06, LegoGyro, 0, 2)
PIDGyro.maxOutput -= 15

leftMotor  = Sensor(sensorType.brick, sensorType.motor, "B")
rightMotor = Sensor(sensorType.brick, sensorType.motor, "C")

Position = PositionTracker(leftMotor, rightMotor)
PositionTracker.gyro = LegoGyro

# --* Drivetrain *--
drive = DriveTrain("B", "C")
IMUWrap.driveRef = drive

# --* Navigation *--


# --* Gate control *--

# ----- interrupt hook -----

def customHook(exctype, value, traceback):
    drive.setAllPowers(0)
    BP.reset_all()
    sys.__excepthook__(exctype, value, traceback)

sys.excepthook = customHook


# --------------------- Brick sensors calibration ----------------------

try:
    Front.update()
    LegoGyro.update()
except brickpi3.SensorError:
    print("Configuring...")
    error = True
    while error:
        sleep(0.1)
        try:
            Front.update()
            LegoGyro.update()
            error = False
        except brickpi3.SensorError:
            error = True
    print("Configured!\n")


# --------------------- Line sensors & IMU calibration --------------------

# biasMulti = 0.2

if (input("Calibrate Magnetic sensor? (y/n): ") == "y"):
    Sensor.Inertial.setBias()
    print("Got IMU Bias")
    # Sensor.Inertial.getNullBand()
    # print("Got IMU Null Band")

    print("Calibration over\n")



# --------------------- function setup --------------------

def turnRight(power):
    drive.turn(True, power)
    drive.update()

def turnLeft(power):
    drive.turn(False, power)
    drive.update()


def driveStraight(base):
    drive.setAllPowers(base)
    Sensor.updateAll()
    PIDGyro.update()

    drive.reduceRight(PIDGyro.output)
    drive.reduceLeft(-PIDGyro.output)

    drive.update()
    

def driveForward(power):
    drive.setAllPowers(power)
    drive.update()

def turnToHeading(target):
    PIDGyro.setTarget(target)
    PIDGyro.goToTarget(turnRight)
    drive.stop()

def turnRelative(distance):
    PIDGyro.setTarget(PIDGyro.target + distance)
    PIDGyro.goToTarget(turnRight)
    drive.stop()


def checkWallDistance():
    if (Left.value < wallDistance):
        drive.reduceLeft(-reduce)
    elif(Right.value < wallDistance):
        drive.reduceRight(-reduce)

def driveToWall():
    Sensor.updateAll()
    while (Front.value > wallDistance * 1.1):
        driveForward(40)
        sleep(period)
        Sensor.updateAll()

    drive.stop()


def runHallway():
    Sensor.updateAll()
    
    startRotation = Sensor.Inertial.rotation
    PIDGyro.setTarget(startRotation)

    while not (Left.value < 40 and Right.value < 40):
        driveStraight(basePower)

        sleep(period)


    Sensor.updateAll()

    while (Left.value < 40 and Right.value < 40):
        driveStraight(basePower)

        checkWallDistance()

        sleep(period)

    turnToHeading(startRotation)
    
    driveToWall()


    if Left.value > 40:
        turnRelative(-90)
    else:
        turnRelative(90)

"""
def runHallway():
    Sensor.updateAll()

    startRotation = Sensor.Inertial.rotation

    while not (Left.value < 40 and Right.value < 40):
        driveForward(basePower)

        checkWallDistance()

        drive.update()
        sleep(period)
        Sensor.updateAll()


    Sensor.updateAll()

    while (Left.value < 40 and Right.value < 40):
        driveForward(basePower)

        checkWallDistance()

        drive.update()
        sleep(period)
        Sensor.updateAll()

    turnToHeading(startRotation)
    
    driveToWall()


    if Left.value > 40:
        turnToHeading(-90)
    else:
        turnToHeading(90)

"""



# ----- cargo control functions -----






# ------

    


# start
delay()