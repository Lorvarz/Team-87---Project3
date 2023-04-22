from PID import *

# --------------------- Sensors and PID setup --------------------------

# --* Sensors *--
magneticSensor = IMUWrap()
Sensor.Inertial = magneticSensor
Left  = Sensor(sensorType.grove, sensorType.ultraSonic, 2)
Right = Sensor(sensorType.grove, sensorType.ultraSonic, 3)
Front = Sensor(sensorType.brick, sensorType.ultraSonicNXT, 1)

IR = Sensor(sensorType.grove, sensorType.analog, 2)
IR2 = Sensor(sensorType.grove, sensorType.analog, 1)

LegoGyro = Sensor(sensorType.brick, sensorType.gyro, 2)

PIDGyro     = PIDController(1.3, 0.005, 0.04, LegoGyro, 0, 2)
PIDGyro.maxOutput -= 10

leftMotor  = Sensor(sensorType.brick, sensorType.motor, "B")
rightMotor = Sensor(sensorType.brick, sensorType.motor, "C")

cargoMotor = Motor("A")
BP.set_motor_limits(cargoMotor.pin, power=20)

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
    try:
        global areaMap
        areaMap.makeMap()
    except:
        pass
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

# --------------------- Map creation --------------------

if (input("Create map? (y/n): ") == "y"):
    scale = int(input("Enter the scale of the tiles: "))
    sideLength = int(input("Enter the number of tiles in a side: "))
    origin = [int(i) for i in input("Enter the origin position: ").split(",")]

    areaMap = Map((sideLength, sideLength))
    areaMap.setup(origin, scale)
    Position.y = origin[0] * scale
    Position.x = origin[1] * scale

    print("Map created")

    initialDirection = Direction(int(input("Enter the initial direction: ")))
    distanceSensors = {Direction.up : Front, 
                       Direction.left : Left, 
                       Direction.right : Right}

    centralNav = Navigator(initialDirection, Position, distanceSensors, IR, magneticSensor, areaMap)

    print("Navigator created")




# --------------------- Line sensors & IMU calibration --------------------

# biasMulti = 0.2

if (input("Calibrate Magnetic sensor? (y/n): ") == "y"):
    Sensor.Inertial.setBias()
    print("Got Magnetic Bias")

if (input("Calibrate IR sensor? (y/n): ") == "y"):
    IR.calibrateBias()
    IR2.calibrateBias()
    print("Calibrated IR sensor")

print("Calibration over\n")



# --------------------- function setup --------------------

def turnRight(power):
    drive.turn(True, power)
    drive.update()

def driveForward(power):
    drive.setAllPowers(power)
    drive.update()

def driveStraight(base):
    drive.setAllPowers(base)
    Sensor.updateAll()
    PIDGyro.update()

    if not avoidWalls():
        drive.reduceRight(PIDGyro.output)
        drive.reduceLeft(-PIDGyro.output)

    drive.update()
    

def turnToHeading(target):
    PIDGyro.setTarget(target)
    PIDGyro.goToTarget(turnRight)
    drive.stop()

def turnRelative(distance):
    PIDGyro.setTarget(Position.rotation + distance)
    PIDGyro.goToTarget(turnRight)
    drive.stop()


def avoidWalls() -> bool:
    if (Left.update() < wallDistance):
        drive.reduceLeft(-reduce)
        return True
    elif(Right.update() < wallDistance):
        drive.reduceRight(-reduce)
        return True
    return False


def driveToWall():
    Sensor.updateAll()
    target = wallDistance * 1.2
    while (abs(Front.value - target)) > 3:
        driveForward((Front.value - target) * 4)
        sleep(period)
        Sensor.updateAll()


def driveDistance(distance):
    select = 1 if Position.direction.value % 2 == 0 else 0
    start = Position.position[select]   

    PIDGyro.setTarget(Position.rotation)

    while (abs(Position.position[select] - start) < distance):
        driveStraight(basePower + (distance - abs(Position.position[select] - start)))
        sleep(period)
        Position.update()

    areaMap.updateCurrentTile()
    if areaMap.currentTile[Position.direction] == edgeType.wall:
        driveToWall()

    drive.stop()
        




def dropCargo():
    cargoMotor.goToPosition(-80)

def closeCargo():
    cargoMotor.goToPosition(0)
    
    



# ----- cargo control functions -----






# ------

    


# start
delay()