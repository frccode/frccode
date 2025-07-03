# Swerve Drive Programming Guide

## Table of Contents

1. [Introduction](#introduction)
4. [Core Concepts](#core-concepts)
5. [Mathematical Foundation](#mathematical-foundation)
6. [Control Systems](#control-systems)
7. [Coordinate Systems](#coordinate-systems)
8. [Implementation Guide](#implementation-guide)
9. [Code Examples](#code-examples)
10. [Troubleshooting](#troubleshooting)

## 1. Introduction
This guide covers the programming concepts needed to implement a swerve drive system using WPILIB.

**What You'll Learn:**
- Fundamental swerve drive concepts
- Mathematical principles behind swerve kinematics
- WPILIB implementation details
- Best practices for control systems
- Common pitfalls and solutions

## 2. Core Concepts

**Holonomic Drive (Swerve)**
- Can move in any direction independent of orientation
- Each wheel can rotate and steer independently
- More complex but more capable

### Control Methods

**Trajectory Control**
- Pre-planned or dynamically generated paths
- Automated movement along specific routes
- Used for autonomous navigation

**Teleop Control**
- Direct driver control via joysticks
- Real-time response to input
- Used during driver-controlled periods

### Swerve Module Components

Each swerve module consists of:
- **Drive Motor**: Controls wheel speed
- **Steer Motor**: Controls wheel direction
- **Encoder**: Provides position feedback
- **Wheel**: The contact point with the ground

The drive motor, steer motor, and encoder are usually team specific and require their own unique instantiation. For the purposes of this article, we will be using the motor instantiations listed below:
- **Drive Motor**: Talon FX Kraken X60
- **Steer Motor**: Talon FX Kraken X60
- **Encoder**: CTRE Cancoder


### Key Classes in WPILIB

### Key Classes in WPILIB

**ChassisSpeeds**

The `ChassisSpeeds` class in relation to the swerve drive represents the desired movement of the robot as a whole. It encapsulates three components:
- **xVelocity**: Forward/backward speed (meters per second)
- **yVelocity**: Left/right speed (meters per second)
- **omegaVelocity**: Rotational speed (radians per second)

This class is commonly used as an input to swerve drive kinematics, which then calculates the necessary wheel speeds and angles for each module to achieve the desired chassis motion. By adjusting these values, you can command the robot to move in any direction and rotate simultaneously.

```java
// Stores target velocities for the entire robot
ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
    xVelocity,    // Forward/backward speed (m/s)
    yVelocity,    // Left/right speed (m/s)
    omegaVelocity // Rotational speed (rad/s)
);
```

**SwerveModuleState**

Represents the state of an individual swerve drive module, encapsulating both its current angle (direction) and speed (velocity). By doing so, it enables coordinated control of all modules to achieve complex robot maneuvers such as strafing, rotating, and omnidirectional movement.

```java
// Stores angle and speed for individual modules
SwerveModuleState moduleState = new SwerveModuleState(
    speed,  // Speed in m/s
    angle   // Angle as Rotation2d
);
```

**SwerveModuleState**

Represents the state of an individual swerve drive module, encapsulating both its current angle (direction) and speed (velocity). By doing so, it enables coordinated control of all modules to achieve complex robot maneuvers such as strafing, rotating, and omnidirectional movement.

```java
// Stores angle and speed for individual modules
SwerveModuleState moduleState = new SwerveModuleState(
    speed,  // Speed in m/s
    angle   // Angle as Rotation2d
);
```

## 3. Mathematical Foundation

### Inverse Kinematics

Swerve drive uses inverse kinematics to calculate individual module states from desired robot motion.

![Swerve Drive Vector Addition Diagram](../images/swerveVector.png)

**Vector Addition Principle:**
For each module, the final velocity vector is the sum of:
1. **Translation vector** (desired robot velocity)
2. **Rotation vector** (tangent to rotation at module position)

**Formula:**
```
V_module = V_translation + ω × r_module
```

Where:
- `V_module` = Final module velocity vector
- `V_translation` = Desired translation velocity
- `ω` = Angular velocity
- `r_module` = Position vector from robot center to module

### Coordinate Transformation

**Field-Oriented Control:**
Convert field coordinates to robot coordinates:
```
x' = x*cos(θ) + y*sin(θ)
y' = y*cos(θ) - x*sin(θ)
```

Where:
- `(x, y)` = Field-relative coordinates
- `(x', y')` = Robot-relative coordinates  
- `θ` = Robot orientation

## 4. Additional Control Concepts

### Speed Desaturation

When calculated module speeds exceed maximum velocity, all modules must be scaled proportionally:

```java
// Find the maximum speed
double maxSpeed = Collections.max(Arrays.asList(speeds));

// If any speed exceeds maximum, scale all down
if (maxSpeed > MAX_SPEED_MPS) {
    double scaleFactor = MAX_SPEED_MPS / maxSpeed;
    for (int i = 0; i < speeds.length; i++) {
        speeds[i] *= scaleFactor;
    }
}
```

### Wheel Optimization

Minimize rotation by choosing the shortest path to target angle:

**Angle Optimization:**
```java
// Calculate angle difference
double angleDifference = targetAngle - currentAngle;

// Normalize to [-180, 180] degrees
while (angleDifference > 180) angleDifference -= 360;
while (angleDifference < -180) angleDifference += 360;

// If > 90 degrees, flip direction and reverse speed
if (Math.abs(angleDifference) > 90) {
    targetAngle += 180;
    speed *= -1;
    angleDifference += (angleDifference > 0) ? -180 : 180;
}
```

### Motor Control

**Steering Control:**
Use PID control for precise angle positioning:
```java
double output = steerPID.calculate(currentAngle, targetAngle);
steerMotor.setVoltage(output);
```

**Drive Control:**
Use feedforward (F) + feedback(PID) for velocity control(PIDF):
```java
double feedforward = driveFeedforward.calculate(targetVelocity);
double feedback = drivePID.calculate(currentVelocity, targetVelocity);
driveMotor.setVoltage(feedforward + feedback);
```

## 5. Coordinate Systems

![WPILIB Swerve Coordinate System](../images/swerveCoord1.png)

Understanding and adhering to the WPILIB coordinate system is crucial because it ensures consistent movement commands, sensor readings, and autonomous routines across the swerve drive. Teams have frequently run into switching alliance issues/direction confusion/unreliable feild control etc. due to misunderstanding the WPILIB coordinate system.

### WPILIB Coordinate System (NWU)

**Field Coordinates:**
- **X-axis**: Points toward opponent alliance (positive = forward)
- **Y-axis**: Points to the left when facing forward  
- **Z-axis**: Points up
- **Rotation**: Counterclockwise positive, starting from +X axis

**Important Notes:**
- All components must use the same coordinate system
- Angles typically range from -180° to +180°
- Field-oriented driving maintains consistent directions regardless of robot orientation

### Alliance Considerations

**Blue Alliance Perspective:**
- Coordinate system origin at blue alliance corner
- Positive X = toward red alliance
- Positive Y = toward left side of field

**Red Alliance Adjustments:**

Actions taken on the alliance side must be mirrored 

 * [TIP] Do not mirror omega turns: Omega turns are specialized maneuvers and should not be mirrored, as this may lead to unintended behavior or incorrect movement patterns.

```java
// Flip coordinates for red alliance in mirrored fields
if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
    xVelocity = -xVelocity;
    yVelocity = -yVelocity;
}
```

## 6. Implementation Guide

### Basic Swerve Drive Class Structure

```java
public class SwerveDrive extends SubsystemBase {
    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final Gyro gyro;
    
    public SwerveDrive() {
        ```java
            // Initialize modules, kinematics, and gyro
            modules = new SwerveModule[] {
                new SwerveModule(1, 2, 11), // Front Left
                new SwerveModule(3, 4, 12), // Front Right
                new SwerveModule(5, 6, 13), // Back Left
                new SwerveModule(7, 8, 14)  // Back Right
            };

            // Use constants for kinematics (see Kinematics Setup section)
            kinematics = Constants.KINEMATICS;

            // Instantiate a Pigeon2 gyro using CTRE's library
            gyro = new Pigeon2(0); // 0 is the CAN ID, adjust as needed
        ```
    }
    
    public void drive(ChassisSpeeds chassisSpeeds) {
        // Convert chassis speeds to module states
        SwerveModuleState[] moduleStates = 
            kinematics.toSwerveModuleStates(chassisSpeeds);
            
        // Desaturate wheel speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(
            moduleStates, MAX_SPEED_MPS);
            
        // Set each module to its target state
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(moduleStates[i]);
        }
    }
}
```

### Teleop Command Implementation

```java
public class TeleopSwerveCommand extends CommandBase {
    private final SwerveDrive swerve;
    private final XboxController controller;
    
    @Override
    public void execute() {
        // Get joystick inputs
        double xSpeed = -controller.getLeftY();  // Forward/backward
        double ySpeed = -controller.getLeftX();  // Left/right  
        double rot = -controller.getRightX();    // Rotation
        
        // Apply deadband
        xSpeed = MathUtil.applyDeadband(xSpeed, 0.1);
        ySpeed = MathUtil.applyDeadband(ySpeed, 0.1);
        rot = MathUtil.applyDeadband(rot, 0.1);
        
        // Scale to max speeds
        xSpeed *= MAX_SPEED_MPS;
        ySpeed *= MAX_SPEED_MPS;
        rot *= MAX_ANGULAR_SPEED_RAD_PER_SEC;
        
        // Create chassis speeds (field-oriented)
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, swerve.getRotation2d());
            
        // Drive the robot
        swerve.drive(chassisSpeeds);
    }
}
```

## 7. Code Examples

### Complete Swerve Module Class

```java
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;

    private final PIDController steerPID;
    private final SimpleMotorFeedforward driveFeedforward;
    private final PIDController drivePID;

    // Constants (replace with your actual values)
    private static final double DRIVE_GEAR_RATIO = 6.75;
    private static final double WHEEL_CIRCUMFERENCE = 0.1016 * Math.PI; // 4" wheel in meters

    public SwerveModule(int driveID, int steerID, int encoderID) {
        driveMotor = new TalonFX(driveID);
        steerMotor = new TalonFX(steerID);
        steerEncoder = new CANcoder(encoderID);

        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        steerMotor.setNeutralMode(NeutralModeValue.Brake);

        steerPID = new PIDController(1.0, 0.0, 0.0);
        steerPID.enableContinuousInput(-Math.PI, Math.PI);

        drivePID = new PIDController(0.1, 0.0, 0.0);
        driveFeedforward = new SimpleMotorFeedforward(0.1, 2.0);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning > 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(
            desiredState, new Rotation2d(getSteerAngle()));

        // Calculate steering output
        double steerOutput = steerPID.calculate(
            getSteerAngle(), state.angle.getRadians());
        steerMotor.set(steerOutput / 12.0); // TalonFX expects [-1, 1] for percent output

        // Calculate drive output
        double driveFF = driveFeedforward.calculate(state.speedMetersPerSecond);
        double driveFB = drivePID.calculate(
            getDriveVelocity(), state.speedMetersPerSecond);
        driveMotor.set((driveFF + driveFB) / 12.0); // Convert voltage to percent output
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveVelocity(),
            new Rotation2d(getSteerAngle())
        );
    }

    private double getSteerAngle() {
        // CANcoder returns [0,1) rotations, convert to radians
        return steerEncoder.getAbsolutePosition().getValue() * 2 * Math.PI;
    }

    private double getDriveVelocity() {
        // TalonFX velocity is in rotations per second, convert to m/s
        double rotationsPerSecond = driveMotor.getVelocity().getValue();
        return rotationsPerSecond * WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    }
}
```

### Kinematics Setup

```java
public class Constants {
    // Robot dimensions (meters)
    public static final double WHEELBASE = 0.6;  // Distance between front/back wheels
    public static final double TRACKWIDTH = 0.6; // Distance between left/right wheels
    
    // Module positions relative to robot center
    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(WHEELBASE / 2, TRACKWIDTH / 2);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(WHEELBASE / 2, -TRACKWIDTH / 2);
    public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-WHEELBASE / 2, TRACKWIDTH / 2);
    public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-WHEELBASE / 2, -TRACKWIDTH / 2);
        
    public static final SwerveDriveKinematics KINEMATICS = 
        new SwerveDriveKinematics(
            FRONT_LEFT_LOCATION,
            FRONT_RIGHT_LOCATION,
            BACK_LEFT_LOCATION,
            BACK_RIGHT_LOCATION
        );
}
```

## 8. Troubleshooting

### Common Issues and Solutions

**Module Not Reaching Target Angle**
- Check encoder calibration and offset values
- Verify PID tuning parameters
- Ensure continuous input is enabled for angle PID

**Robot Drifting During Movement**
- Calibrate gyroscope properly
- Check for mechanical issues (loose wheels, etc.)
- Verify coordinate system consistency

**Jerky or Unstable Movement**
- Reduce PID gains, especially derivative
- Add input filtering/deadbands
- Check for sensor noise

**Modules Fighting Each Other**
- Ensure all modules use same coordinate system
- Verify kinematics setup matches physical layout
- Check for inverted motors or encoders

### Debug Tools

**Module State Monitoring:**
```java
public void updateTelemetry() {
    for (int i = 0; i < modules.length; i++) {
        SwerveModuleState state = modules[i].getState();
        SmartDashboard.putNumber("Module " + i + " Speed", state.speedMetersPerSecond);
        SmartDashboard.putNumber("Module " + i + " Angle", state.angle.getDegrees());
    }
}
```

**Field-Relative Position Tracking:**
```java
private final SwerveDriveOdometry odometry;

public void updateOdometry() {
    odometry.update(gyro.getRotation2d(), getModuleStates());
    SmartDashboard.putString("Robot Pose", odometry.getPoseMeters().toString());
}
```

### Best Practices

1. **Start Simple**: Begin with basic teleop control before adding autonomous features
2. **Test Incrementally**: Test each module individually before running full swerve
3. **Use Simulation**: WPILIB's swerve simulation helps debug without hardware
4. **Monitor Performance**: Always log module states and robot pose for debugging
5. **Consistent Units**: Use meters and radians throughout for consistency with WPILIB

## 9. Additional Resources

- [WPILIB Swerve Drive Documentation](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html)
- [Team 364's Swerve Guide](https://www.team364.org/programming)
- [Chief Delphi Swerve Resources](https://www.chiefdelphi.com/)
- [SwerveLib by Team 5190](https://github.com/5190GreenHopeRobotics/5190-Library)

---

*This guide is based on Team 2637's swerve drive training presentation and includes additional implementation details for new programmers.*