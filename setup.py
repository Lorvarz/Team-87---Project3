from PID import *

# --------------------- Sensors and PID setup --------------------------

# --* Sensors *--
magneticSensor = IMUWrap()
Sensor.Inertial = magneticSensor
Left  = Sensor(sensorType.grove, sensorType.ultraSonic, 2)
Right = Sensor(sensorType.grove, sensorType.ultraSonic, 3)
Front = Sensor(sensorType.brick, sensorType.ultraSonicNXT, 1)

IR = Sensor(sensorType.grove, sensorType.analog, 1)
# IR2 = Sensor(sensorType.grove, sensorType.analog, 1)

LegoGyro = Sensor(sensorType.brick, sensorType.gyro, 2)

PIDGyro     = PIDController(1.45, 0.001, 0.04, LegoGyro, 0, 2.5)
PIDGyro.maxOutput -= 10
# PIDGyro.maxOutput = 60

PIDStraight = PIDController(17.5, 0, 0.005, Left, 0, 2)

leftMotor  = Sensor(sensorType.brick, sensorType.motor, "B")
rightMotor = Sensor(sensorType.brick, sensorType.motor, "C")

cargoMotor = Motor("A")
BP.set_motor_limits(cargoMotor.pin, power=20)

Position = PositionTracker(leftMotor, rightMotor)
PositionTracker.gyro = LegoGyro

# --* Drivetrain *--
drive = DriveTrain("B", "C")
IMUWrap.driveRef = drive


# ----- interrupt hook -----

def customHook(exctype, value, traceback):
    drive.setAllPowers(0)
    BP.reset_all()
    try:
        global areaMap
        areaMap.makeMap()
        # areaMap.printMapEdges()
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
    scale = 40
    sideLength = tuple([int(num) for num in (input("Enter the number of tiles in a side: ")).split(",")])
    origin = [int(i) for i in input("Enter the origin position: ").split(",")]

    areaMap = Map(sideLength)
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
    magneticSensor.setBias()
    print("Got Magnetic Bias")
    input("Place robot next to beacon")
    magneticSensor.update()
    magneticSensor.magThreshold = magneticSensor.magneticMagnitude * 1.15
else:
    magneticSensor.magThreshold = 9999

if (input("Calibrate IR sensor next to beacon? (y/n): ") == "y"):
    IR.calibrateBias(multi = 0.95)
    print("Calibrated IR sensor")
else:
    IR.biased = True
    IR.bias = -9999

print("Calibration over\n")



# --------------------- function setup --------------------

def turnRight(power):
    drive.turn(True, power)
    drive.update()

def driveForward(power):
    drive.setAllPowers(power)
    drive.update()

def driveStraight(base, orthoDistance, adjustSign):
    """Drives straight using the gyro and the side distance sensors"""
    drive.setAllPowers(base)
    Sensor.updateAll()
    PIDGyro.update()
    PIDStraight.update(value = orthoDistance)

    drive.adjustPowers(PIDStraight.output * adjustSign)
    avoidWalls()

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
    if (Left.value < wallDistance):
        drive.reduceLeft(-reduce)
        return True
    elif(Right.value < wallDistance):
        drive.reduceRight(-reduce)
        return True
    return False


def driveToWall():
    Sensor.updateAll()
    target = 18
    while (abs(Front.value - target)) > 3:
        driveForward((Front.value - target) * 4)
        sleep(period)
        Sensor.updateAll()


def driveDistance(distance):
    """Drives a certain distance in cm"""
    select = 1 if Position.direction.value % 2 == 0 else 0
    start = Position.position[select]   
    orthogonalStart = areaMap.currentTile.position[select] * areaMap.scale

    angleTarget = (Position.rotation + 45) // 90 * 90 

    PIDGyro.setTarget(angleTarget)
    adjustSign = 1 if Position.direction == Direction.up or Position.direction == Direction.left else -1

    while (abs(Position.position[select] - start) < distance):
        driveStraight(basePower + (distance - abs(Position.position[select] - start)), 
                      Position.position[int(not select)] - orthogonalStart,
                      adjustSign)
        sleep(period)
        Position.update()

    areaMap.updateCurrentTile()
    if areaMap.currentTile[Position.direction] == edgeType.wall:
        driveToWall()

    Position.update()
    angleTarget = (Position.rotation + 45) // 90 * 90 
    turnRelative(angleTarget - Position.rotation)

    drive.stop()
        

def dropCargo():
    cargoMotor.goToPosition(-80)

def closeCargo():
    cargoMotor.goToPosition(0)
    
    
# start
delay()