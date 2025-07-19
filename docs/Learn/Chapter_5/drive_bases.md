# Drive Base in FRC Programming

## Types of Drive Bases
> - **Differential(aka West Coast Drive / Tank Drive)** enables simple maneuverability using two mirrored drivetrains(sets of 2-3 wheels powered by a set of motors and gearboxes) that independently power each side of the drive base. This allows for simple yet effective manuvering with speed differences, strong pushing power, and a ease of control system implementation.

> - **Swerve drive** enables ominidirectional drivebase control, allowing robots drive in ways that typical drivebases cannot do (ie. spin in place, sideways, diagonally, drive while spinning). It has become the dominant drivebase in FRC since 2022–2023, giving teams a significant mobility advantage.

> - **Mecanum drive** is four wheel drive system in which every wheel is a "mecanum wheel"(pictured below).<!-- TODO Add mecanum wheel picture-->
This special wheel enables ominidrectional movement that includes strafing directly sideways or diagonally.

> - **H Drive** is a differential drive with an added fith wheel(usually an omniwheel) at the center of the drivebase. This extra wheel enables sideways strafe motion.

> - **Other Swerve Drives** While the most common FRC drive bases are covered above, there are several other unique or experimental designs used by teams for specific strategic advantages. **Butterfly Drive** and **Octanum Drive** are hybrid systems that allow switching between traction and omnidirectional movement—Butterfly uses omni and traction wheels, while Octanum uses mecanum and traction wheels. **Kiwi Drive** uses three omni wheels in a triangular layout for holonomic movement. **Crab Drive** mounts all wheels on steering modules so the robot can move in any direction without changing orientation, with all wheels turning together. **Ackermann Steering** mimics car-style steering with only the front wheels turning, and is rare in FRC. Teams also sometimes create custom or hybrid drives to meet unique game challenges, combining features from multiple systems or using nonstandard wheel arrangements. These drive bases are less common but demonstrate the creativity and engineering diversity found in FRC.

## Focus of This Guide

This guide will primarily focus on programming swerve drives and tank drives (also known as differential or West Coast drive). These systems are most commonly used in competition due to their versatility, performance, and prevalence in modern FRC games. The following chapters will provide detailed explanations, code examples, and best practices for implementing both swerve and tank drive systems in your robot code.

## Key Concepts Across All Drivebases
### Java Suppliers and FRC Usage

A `Supplier<T>` in Java is a functional interface that represents a function with no arguments that returns a value of type `T`. Suppliers are commonly used for deferred or dynamic value retrieval, such as reading sensor data or getting the latest state of a subsystem.

**FRC Example:**  
In FRC robot code, Suppliers are often used to pass joystick or sensor values into commands or subsystems. For example, when creating a command to drive a robot, you might use `DoubleSupplier` (a primitive specialization of `Supplier<Double>`) to provide the latest joystick input each time the command runs:

```java
// Example: Passing joystick values as suppliers to a drive command
DoubleSupplier forward = () -> driverController.getLeftY();
DoubleSupplier turn = () -> driverController.getRightX();

DriveCommand driveCommand = new DriveCommand(driveSubsystem, forward, turn);
```

This approach ensures the command always uses the most recent joystick values, making the robot responsive to operator input.

### Organization of Drive Base Subsystems, Commands, and Command Calls in FRC

In the FRC Command-Based framework, robot code is organized into subsystems and commands to promote modularity and clarity:

#### 1. **Subsystems**
A *subsystem* represents a physical part of the robot (e.g., the drive base). It contains methods to control hardware (motors, sensors) and maintains the state of that mechanism.

```java
public class DriveSubsystem extends SubsystemBase {
    // Motor controllers and sensors declared here

    public void drive(double forward, double turn) {
        // Code to set motor outputs
    }
}
```

#### 2. **Commands**
A *command* defines a specific robot action or behavior, often using one or more subsystems. For a drive base, a command might continuously read joystick values and call the drive method.

```java
public class DriveCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier forward, turn;

    public DriveCommand(DriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier turn) {
        this.driveSubsystem = subsystem;
        this.forward = forward;
        this.turn = turn;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.drive(forward.getAsDouble(), turn.getAsDouble());
    }
}
```

#### 3. **Command Calls (Binding)**
Commands are scheduled or bound to triggers (like joystick buttons or default behaviors) in the `RobotContainer` class.

```java
driveSubsystem.setDefaultCommand(
    new DriveCommand(driveSubsystem, 
                     () -> driverController.getLeftY(), 
                     () -> driverController.getRightX())
);
```

#### **Diagram: Command-Based Structure for Drive Base**

```mermaid
graph TD
    A[Driver Controller] -->|Supplies values| B[DriveCommand]
    B -->|Calls| C[DriveSubsystem]
    C -->|Controls| D[Motors/Sensors]
```

- **Driver Controller**: Provides input (joystick values).
- **DriveCommand**: Reads input, calls drive methods.
- **DriveSubsystem**: Implements hardware control.
- **Motors/Sensors**: Physical hardware on the robot.

This structure separates hardware logic from robot behavior, making code easier to maintain and extend.

## Simulator

Below is an interactive simulator showing the difference between drivebases.


<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>FRC Drive Base Simulator</title>
    <link rel="stylesheet" href="simulator.css">
</head>
<body>
    <div class="simulator-container">
        <h1 class="title">🤖 FRC Drive Base Simulator</h1>
        <div class="controls-section">
            <div class="control-panel">
                <div class="control-group">
                    <h3>Drive Type Selection</h3>
                    <div class="drive-type-selector">
                        <button class="drive-type-btn active" onclick="setDriveType('tank')">Tank Drive</button>
                        <button class="drive-type-btn" onclick="setDriveType('swerve')">Swerve Drive</button>
                    </div>
                </div>
                <div class="input-display">
                    <div class="input-group">
                        <h4>Drive Inputs</h4>
                        <div class="input-label">Forward/Backward</div>
                        <div class="input-bar">
                            <div class="input-bar-fill" id="forwardBar"></div>
                        </div>
                        <div class="input-label">Left/Right Turn</div>
                        <div class="input-bar">
                            <div class="input-bar-fill" id="turnBar"></div>
                        </div>
                    </div>
                    <div class="input-group" id="swerveInputs" style="display: none;">
                        <h4>Swerve Inputs</h4>
                        <div class="input-label">Strafe (Left/Right)</div>
                        <div class="input-bar">
                            <div class="input-bar-fill" id="strafeBar"></div>
                        </div>
                        <div class="input-label">Rotation</div>
                        <div class="input-bar">
                            <div class="input-bar-fill" id="rotationBar"></div>
                        </div>
                    </div>
                </div>
                <button class="reset-btn" onclick="resetRobot()">🔄 Reset Position</button>
            </div>
        </div>
        <div class="field-container">
            <canvas id="gameField" width="800" height="400"></canvas>
            <div class="robot-info">
                <div class="telemetry" id="telemetry">
                    <div>X: <span id="posX">0.0</span>m</div>
                    <div>Y: <span id="posY">0.0</span>m</div>
                    <div>Angle: <span id="angle">0.0</span>°</div>
                    <div>Speed: <span id="speed">0.0</span>m/s</div>
                </div>
            </div>
        </div>
        <div class="instructions">
            <h3>🎮 Controls</h3>
            <p><strong>Movement:</strong></p>
            <div class="key-group">
                <span class="key">W</span><span class="key">A</span><span class="key">S</span><span class="key">D</span> - WASD movement
            </div>
            <div class="key-group">
                <span class="key">↑</span><span class="key">←</span><span class="key">↓</span><span class="key">→</span> - Arrow keys
            </div>
            <br>
            <p><strong>Swerve-specific:</strong></p>
            <div class="key-group">
                <span class="key">Q</span><span class="key">E</span> - Rotate left/right
            </div>
            <div class="key-group">
                <span class="key">Z</span><span class="key">C</span> - Strafe left/right
            </div>
            <p><strong>Tank Drive:</strong> Uses differential steering - forward/back + left/right turn</p>
            <p><strong>Swerve Drive:</strong> Full omnidirectional movement with independent translation and rotation</p>
        </div>
    </div>

    <script src="simulator.js"></script>
</body>
</html>

## Where to go Next?

Choose the following section based off the drivebase you plan to program:

[Swerve Control](./swerve_control.md)


[Tank Drive Control](./differential_control.md) 



## Article
For more information on drive bases. Please check out:
> https://docs.revrobotics.com/frc-kickoff-concepts/2023/drivetrains