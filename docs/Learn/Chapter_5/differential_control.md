# Differential (Tank) Drive Programming Guide

## Table of Contents

1. [Introduction](#introduction)
2. [Core Concepts](#core-concepts)
3. [Mathematical Foundation](#mathematical-foundation)
4. [Control Systems](#control-systems)
5. [Coordinate Systems](#coordinate-systems)
6. [Implementation Guide](#implementation-guide)
7. [Code Examples](#code-examples)
8. [Troubleshooting](#troubleshooting)
9. [Additional Resources](#additional-resources)

## 1. Introduction
This guide covers the programming concepts needed to implement a differential (tank) drive system using WPILIB.

**What You'll Learn:**
- Fundamental tank drive concepts
- Mathematical principles behind tank drive kinematics
- WPILIB implementation details
- Best practices for control systems
- Common pitfalls and solutions

## 2. Core Concepts

**Differential (Tank) Drive**
A differential (tank) drive system consists of two sets of wheels—left and right—that are powered independently. The robot turns by varying the speed between the two sides, allowing for precise control of direction. This configuration is simple, robust, and widely used in FRC due to its reliability and ease of implementation.

### Control Methods

**Trajectory Control**
- Pre-planned or dynamically generated paths
- Used for autonomous navigation

**Teleop Control**
- Direct driver control via joysticks or tank controls
- Real-time response to input

### Key Components
- **Left Motors**: Drive the left side
- **Right Motors**: Drive the right side
- **Encoders**: Measure wheel rotation for distance/speed
- **Gyro**: Measures robot heading (optional but recommended)


### Key Classes in WPILIB

Listed here are the objects/libraries from wpilib that are used to help with writing a diffierential drive. 

**DifferentialDrive**
```java
// Controls left and right motors
DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);
```


**ChassisSpeeds**
```java
// Stores target velocities for the robot
ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
    xVelocity,    // Forward/backward speed (m/s)
    0.0,          // No strafe for tank drive
    omegaVelocity // Rotational speed (rad/s)
);
```

## 3. Mathematical Foundation

### Kinematics

In the context of FRC drives, **kinematics** refers to the mathematical relationships that describe how the robot's wheel speeds translate to its overall movement (position, velocity, and rotation) on the field, and vice versa. For differential (tank) drive systems, kinematics allows you to:

- Determine the robot's forward and rotational velocities based on the speeds of the left and right wheels (forward kinematics).
- Calculate the required wheel speeds to achieve a desired robot motion (inverse kinematics).

Understanding kinematics is essential for tasks such as autonomous path following, odometry, and precise control of the robot's movement.

**Tank drive kinematics relate left/right wheel speeds to robot motion.**

**Forward Kinematics:**
- Calculate robot velocity from left/right wheel speeds

**Inverse Kinematics:**
- Calculate left/right wheel speeds from desired robot velocity

**Formulas:**
```
v_left = v - ω * (trackwidth / 2)
v_right = v + ω * (trackwidth / 2)
```
Where:
- `v` = Forward velocity (m/s)
- `ω` = Angular velocity (rad/s)
- `trackwidth` = Distance between left and right wheels (meters)

## 4. Control Systems

### Speed Control
Use PID control for precise velocity control of each side. Place this code within your drive subsystem periodic loop:
```java
double leftOutput = leftPID.calculate(currentLeftSpeed, targetLeftSpeed);
double rightOutput = rightPID.calculate(currentRightSpeed, targetRightSpeed);
leftMotor.setVoltage(leftOutput);
rightMotor.setVoltage(rightOutput);
```

### Arcade vs. Tank Drive
- **Tank Drive**: Separate joystick for each side
- **Arcade Drive**: One joystick for forward/backward, one for turning

**Arcade Drive Example:**
```java
drive.arcadeDrive(forward, rotation);
```

**Tank Drive Example:**
```java
drive.tankDrive(leftSpeed, rightSpeed);
```

## 5. Coordinate Systems

WPILIB uses the same coordinate system as swerve drive. Consistency is important for autonomous routines and sensor integration.

## 6. Implementation Guide

### Basic Tank Drive Class Structure
Implement this class as a template subsystem for your differential drive. This class is the factory that will run your differential drive.

Call the drive method periodically to set forward, backwards, and rotational speeds.
```java
public class TankDrive extends SubsystemBase {
    private final DifferentialDrive drive;
    private final MotorController leftMotor;
    private final MotorController rightMotor;
    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final Gyro gyro;

    public TankDrive() {
        // Initialize motors, encoders, and gyro
        drive = new DifferentialDrive(leftMotor, rightMotor);
    }

    public void drive(double forward, double rotation) {
        drive.arcadeDrive(forward, rotation);
    }
}
```

### Teleop Command Implementation

Use this command template for controlling your differential drive using suppliers. Refer to [Suppliers](../drive_bases.md#suppliers).
```java
public class TeleopTankCommand extends CommandBase {
    private final TankDrive tankDrive;
    private final Supplier<Double> forwardSupplier;
    private final Supplier<Double> rotationSupplier;

    public TeleopTankCommand(
        TankDrive tankDrive,
        Supplier<Double> forwardSupplier,
        Supplier<Double> rotationSupplier
    ) {
        this.tankDrive = tankDrive;
        this.forwardSupplier = forwardSupplier;
        this.rotationSupplier = rotationSupplier;
        addRequirements(tankDrive);
    }

    @Override
    public void execute() {
        double forward = MathUtil.applyDeadband(forwardSupplier.get(), 0.1);
        double rotation = MathUtil.applyDeadband(rotationSupplier.get(), 0.1);
        tankDrive.drive(forward, rotation);
    }
}
```
### Example Usage in RobotContainer

```java
public class RobotContainer {
    private final TankDrive tankDrive = new TankDrive();
    private final CommandXboxController driverController = new CommandXboxController(0);

    public RobotContainer() {
        // Set the default command for teleop driving
        tankDrive.setDefaultCommand(
            new TeleopTankCommand(
                tankDrive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getRightX()
            )
        );
    }
}
```


## 7. Code Examples

### Encoder Distance Calculation
```java
// Returns distance in meters
public double getLeftDistance() {
    return leftEncoder.getDistance();
}
public double getRightDistance() {
    return rightEncoder.getDistance();
}
```

### Odometry Setup
```java
private final DifferentialDriveOdometry odometry;

public void updateOdometry() {
    odometry.update(gyro.getRotation2d(), getLeftDistance(), getRightDistance());
    SmartDashboard.putString("Robot Pose", odometry.getPoseMeters().toString());
}
```

## 8. Troubleshooting

**Robot Drifts or Turns Unexpectedly**
- Check for motor inversion or wiring issues
- Verify encoder and gyro calibration

**Inaccurate Distance or Heading**
- Ensure encoder conversion factors are correct
- Calibrate gyro

**Jerky or Unstable Movement**
- Tune PID gains
- Add input deadbands

## 9. Additional Resources

- [WPILIB Differential Drive Documentation](https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html)
- [WPILIB Odometry](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-odometry.html)
- [FRC Tank Drive Examples](https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/differentialdrivebot)

---

