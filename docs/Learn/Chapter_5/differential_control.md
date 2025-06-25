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
- Two sets of wheels (left and right) powered independently
- Robot turns by varying speed between sides
- Simple, robust, and widely used in FRC

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

Tank drive kinematics relate left/right wheel speeds to robot motion.

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
Use PID control for precise velocity control of each side:
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
```java
public class TeleopTankCommand extends CommandBase {
    private final TankDrive tankDrive;
    private final XboxController controller;

    @Override
    public void execute() {
        double forward = -controller.getLeftY();
        double rotation = -controller.getRightX();
        forward = MathUtil.applyDeadband(forward, 0.1);
        rotation = MathUtil.applyDeadband(rotation, 0.1);
        tankDrive.drive(forward, rotation);
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

*This guide is based on FRC programming best practices and includes implementation details for new programmers.*
