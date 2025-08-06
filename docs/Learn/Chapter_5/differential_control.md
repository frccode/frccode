# Differential (Tank) Drive Programming Guide

## Table of Contents

1. [Introduction](#introduction)
2. [Core Concepts](#core-concepts)
3. [Mathematical Foundation](#mathematical-foundation)
4. [Additional Control Concepts](#additional-control-concepts)
5. [Coordinate Systems](#coordinate-systems)
6. [Implementation Guide](#implementation-guide)
7. [Troubleshooting](#troubleshooting)
8. [Additional Resources](#additional-resources)

## 1. Introduction

This guide covers the programming concepts needed to implement a differential (tank) drive system using WPILIB. Understanding how joystick inputs are translated into motor commands will help you debug common programming issues.

> **Note:** While WPILIB provides high-level abstractions for tank drive, learning the underlying principles will give you more control and flexibility in your robot code.

**What You'll Learn:**
- Fundamental tank drive concepts
- Mathematical principles behind tank drive kinematics
- WPILIB implementation details
- Best practices for control systems
- Common pitfalls and solutions

## 2. Core Concepts

Below are some common terms associated with differential (tank) drive:

### Control Definitions

* **Differential (Tank) Drive**
    - Two sets of wheels (left and right) powered independently
    - Robot turns by varying speed between sides
    - Simple, robust, and widely used in FRC

* **Teleop Control**
    - Direct driver control via joysticks or tank controls
    - Real-time response to input

### Key Components

* **Left Motors**: Drive the left side
* **Right Motors**: Drive the right side
* **Encoders**: Measure wheel rotation for distance/speed
* **Gyro**: Measures robot heading (optional but recommended)

### Key Classes in WPILIB

#### **DifferentialDrive**

Controls left and right motors for tank drive.

```java
DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);
```

#### **ChassisSpeeds**

Represents the desired movement of the robot as a whole.

```java
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

Below is a step-by-step guide for implementing a basic differential (tank) drive subsystem and teleop command.

### Step 1: Create Your Tank Drive Subsystem

Start by building a subsystem class to encapsulate all hardware and drive logic. This class should manage the motors, encoders, and gyro, and provide a method to drive the robot.

**Key Features:**
- Holds references to motors, encoders, and gyro
- Provides a `drive` method for controlling movement
- Initializes all hardware in the constructor

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

### Step 2: Implement a Teleop Command

Create a command class to handle teleoperated driving. This command reads joystick values using suppliers, applies a deadband for smoother control, and calls the drive method.

**Key Steps:**
- Accepts suppliers for forward and rotation input
- Applies deadband to filter out small joystick noise
- Calls the subsystem's drive method each cycle

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

### Step 3: Integrate into

Set up your subsystem and command in `RobotContainer` to enable teleop driving. Use suppliers to connect joystick axes to your command.

**Key Steps:**
- Instantiate the subsystem and controller
- Set the teleop command as the default for the subsystem
- Use lambda expressions to supply joystick values

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


## 7. Extra Features

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

## 8. Troubleshooting and Possible Fixes

**Robot Drifts or Turns Unexpectedly**
- Verify encoder and gyro readings

**Inaccurate Distance or Heading**
- Ensure encoder conversion factors/instantiations are correct

**Jerky or Unstable Movement**
- Add input deadbands
- Add 0.5 or 0.2 scale factors to decrease "sensitivity"

## 9. Additional Resources

- [WPILIB Differential Drive Documentation](https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html)
- [WPILIB Odometry](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-odometry.html)
- [FRC Tank Drive Examples](https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/differentialdrivebot)

---

