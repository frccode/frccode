
# Command-Based Robot Programming Guide

> **Note:** Much of command-based programming has been extensively documented on the [WPILib Docs](https://docs.wpilib.org/en/stable/docs/software/commandbased/). This guide is meant to simplify explanations and provide a step-by-step approach for users creating their first command-based program. For every topic, we will link the relevant WPILib reference documentation.

## 1. Understanding Command-Based Programming

### What is Command-Based Programming?

Command-based programming is a declarative paradigm that helps you organize your WPILib project and execute robot code logic in an organized fashion. 

### Why Use Command-Based Programming?

- **Modularity**: Breaks down robot actions into small, reusable commands and subsystems, making code easier to maintain and extend.
- **Independent Development**: Allows each subsystem and command to be developed and tested separately, helping teams manage complexity as the robot grows.
- **Supports Parallel and Sequential Actions**: Enables more sophisticated robot behaviors by naturally supporting concurrent and ordered operations.
- **Cleaner Code and Collaboration**: Syphoned abstracted logic between hardware, high level multi mechanism logic, and lower level single mechanism logic leads to "less boilerplate" code, and easier debugging.

The command-based pattern is based around two core abstractions: **commands** and **subsystems**.

### Key Concepts

**Commands**: Commands represent actions the robot can take. Commands run when scheduled, until they are interrupted or their end condition is met. Commands are very recursively composable: commands can be composed to accomplish more-complicated tasks.

**Subsystems**: Subsystems represent independently-controlled collections of robot hardware (such as motor controllers, sensors, pneumatic actuators, etc.) that operate together. Subsystems back the resource-management system of command-based: only one command can use a given subsystem at the same time.

[WPILIB Command based explanation](https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html)

[Timed vs Command-Based Discussion](https://www.chiefdelphi.com/t/timed-vs-command-based/418622)

## 2. Setting Up Your Project Structure

### Creating a Command-Based Project

1. **In WPILib Visual Studio Code**:
    - Execute the command "Create a new project"
    - Select "Template" → "java" → "Command Robot"
    - Enter your project information
    - Create the project

### Project Structure Overview

A standard template for a command-based robot project is included in the WPILib examples repository. The root package/directory generally will contain four classes:

```
src/main/java/frc/robot/
├── Main.java              // Main robot application (don't modify)
├── Robot.java             // Main control flow
├── RobotContainer.java    // Robot setup and bindings
├── Constants.java         // Global constants
├── subsystems/            // All subsystem classes
│   └── ArmSubsystem.java
└── commands/              // All command classes
     └── MoveArmCommand.java
```

### Key Files Explained

- **Main.java**: The entry point for your robot program (Java only - don't modify)
- **Robot.java**: Handles the main control flow and periodic updates
- **RobotContainer.java**: Where you define subsystems, commands, and button bindings
- **Constants.java**: Store all your robot's constants in one place

### Reference material
[Structuring a Command-Based Robot Project](https://docs.wpilib.org/en/stable/docs/software/commandbased/structuring-command-based-project.html)

## 3. Setting up a Subsystem

### What Are Subsystems?

Subsystems are the basic unit of robot organization in the command-based paradigm. A subsystem is an abstraction for a collection of robot hardware that operates together as a unit.

### Create a Subsystem

The recommended method to create a subsystem for most users is to subclass the abstract SubsystemBase class.

**Basic Subsystem Structure:**

```java
public class ArmSubsystem extends SubsystemBase {
     // Hardware components
     private final PWMSparkMax armMotor = new PWMSparkMax(Constants.ArmConstants.ARM_MOTOR_PORT);
     private final Encoder armEncoder = new Encoder(0, 1);
     
     /** Creates a new ArmSubsystem. */
     public ArmSubsystem() {
          // Initialize hardware here
     }
     
     @Override
     public void periodic() {
          // This method is called once per scheduler run
          // Use for continuous monitoring/updates
     }
     
     // Subsystem methods that commands will call
     public void setArmSpeed(double speed) {
          armMotor.set(speed);
     }
     
     public double getArmPosition() {
          return armEncoder.getDistance();
     }
     
     public void resetArmEncoder() {
          armEncoder.reset();
     }
     
     // Command factory methods
     public Command moveArmCommand(double speed) {
          return runOnce(() -> setArmSpeed(speed));
     }
     
     public Command stopArmCommand() {
          return runOnce(() -> setArmSpeed(0));
     }
}
```

### Subsystem Best Practices

1. **Encapsulation**: Hide hardware details inside the subsystem
2. **Single Responsibility**: Each subsystem should control one functional area (e.g., the arm should only be responsible for arm movement)
3. **Provide Clean Interface**: Offer methods that commands can easily use
4. **Use periodic()**: For continuous monitoring and updates

### Reference material

[Subsystems](https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html)

## 4. Setting up Commands

### Reference Material
- [Commands](https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html)
- [Composing Complex Commands](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-compositions.html)

### What Are Commands?

Commands represent actions the robot can take. Commands run when scheduled, until they are interrupted or their end condition is met.

### Command Lifecycle

Every command, including Built-In, factory, and class-types, has four key methods:

1. **initialize()**: Called once when the command starts
2. **execute()**: Called repeatedly while the command runs
3. **isFinished()**: Returns true when the command should end
4. **end()**: Called once when the command finishes

### Using Built-in Command Types

The command-based library includes many pre-written command types. Through the use of lambdas, these commands can cover almost all use cases and teams should rarely need to write custom command classes.

### Common Command Types and When to Use Them

Below are the most common command types in WPILib, each with a description and guidance on when to use them:

1. **InstantCommand**
     - **Use when:** You want to perform a quick, one-time action that completes immediately (e.g., reset the arm encoder).
     - **Example:**
        ```java
        new InstantCommand(() -> armSubsystem.resetArmEncoder(), armSubsystem);
        ```

2. **RunCommand**
     - **Use when:** You want to run code continuously while the command is scheduled, and stop when it ends (e.g., move the arm while a button is held).
     - **Example:**
        ```java
        new RunCommand(() -> armSubsystem.setArmSpeed(0.5), armSubsystem);
        ```

3. **StartEndCommand**
     - **Use when:** You need to run code while the command is active, and a different action when it ends (e.g., move the arm up on press, stop on release).
     - **Example:**
        ```java
        new StartEndCommand(
                () -> armSubsystem.setArmSpeed(0.5),
                () -> armSubsystem.setArmSpeed(0),
                armSubsystem
        );
        ```

4. **FunctionalCommand**
     - **Use when:** You want full control over initialization, execution, ending, and completion conditions, but don’t want to write a full class (e.g., move the arm until it reaches a certain position).
     - **Example:**
        ```java
        new FunctionalCommand(
                () -> armSubsystem.resetArmEncoder(), // initialize
                () -> armSubsystem.setArmSpeed(0.5),  // execute
                interrupted -> armSubsystem.setArmSpeed(0), // end
                () -> armSubsystem.getArmPosition() >= 100, // isFinished
                armSubsystem
        );
        ```

5. **Command Classes (Extending CommandBase)**
     - **Use when:** The command is complex, reused, or needs its own state/fields (e.g., move the arm to a specific position).
     - **Example:**
        ```java
        public class MoveArmToPosition extends CommandBase {
                private final ArmSubsystem arm;
                private final double targetPosition;

                public MoveArmToPosition(ArmSubsystem arm, double targetPosition) {
                      this.arm = arm;
                      this.targetPosition = targetPosition;
                      addRequirements(arm);
                }

                @Override
                public void initialize() {
                      arm.resetArmEncoder();
                }

                @Override
                public void execute() {
                      arm.setArmSpeed(0.5);
                }

                @Override
                public boolean isFinished() {
                      return arm.getArmPosition() >= targetPosition;
                }

                @Override
                public void end(boolean interrupted) {
                      arm.setArmSpeed(0);
                }
        }
        ```

6. **Command Factories (Factory Methods)**
     - **Use when:** You want to quickly create simple commands using static factory methods for common patterns.  
     - **Note:** These factory methods (`runOnce`, `run`, `waitSeconds`, etc.) are only available within a `SubsystemBase` subclass (i.e., inside your subsystem class).
     - **Examples (inside ArmSubsystem):**
        ```java
        // Inside your ArmSubsystem class

        // Run a method once and finish immediately
        public Command resetEncoderCommand() {
             return runOnce(this::resetArmEncoder);
        }

        // Run a method repeatedly until interrupted
        public Command holdArmCommand() {
             return run(() -> setArmSpeed(0.1));
        }

        // Run until a condition is met
        public Command moveArmUntilPositionCommand(double target) {
             return run(() -> setArmSpeed(0.5))
                  .until(() -> getArmPosition() >= target);
        }

        // Wait for a specified time
        public Command waitTwoSecondsCommand() {
             return waitSeconds(2.0);
        }
        ```

> **Tip:**  
> Start with the simplest command type that fits your need. Use full command classes only for complex or reusable behaviors.

---

#### Summary Table

| Command Type      | Use For                   |
|-------------------|--------------------------|
| InstantCommand    | One-shot actions         |
| RunCommand        | Continuous actions       |
| StartEndCommand   | Start/stop paired actions|
| FunctionalCommand | Custom lifecycle logic   |
| Command Class     | Complex or reusable cmds |

> **Tip:**  
> Start with simpler command types. Use full classes for more complex or stateful behaviors.  
> See [WPILib Command Types](https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html#command-types) for more details.

## 5. Chaining Commands in Command Compositions

### Building Complex Commands

Individual commands are capable of accomplishing a large variety of robot tasks, but the simple three-state format can quickly become cumbersome when more advanced functionality requiring extended sequences of robot tasks or coordination of multiple robot subsystems is required.

### Sequential Commands

Run commands one after another:

```java
// Using the sequence factory
Command autoSequence = Commands.sequence(
     runOnce(() -> armSubsystem.setArmSpeed(0.5)),
     waitSeconds(0.5),
     run(() -> armSubsystem.setArmSpeed(0.2)).withTimeout(2.0),
     runOnce(() -> armSubsystem.setArmSpeed(0))
);
```

### Parallel Commands

There are three types of parallel compositions, differing based on when the composition finishes:

```java
// All commands must finish
Command parallelAll = Commands.parallel(
     runOnce(() -> armSubsystem.setArmSpeed(0.5)),
     runOnce(() -> intakeSubsystem.deploy())
);

// Finish when any command finishes
Command raceGroup = Commands.race(
     run(() -> driveSubsystem.arcadeDrive(0.5, 0)),
     waitSeconds(3.0)
);

// Run commands alongside each other
Command driveAndMoveArm = Commands.run(() -> driveSubsystem.arcadeDrive(0.5, 0))
     .alongWith(run(() -> armSubsystem.setArmSpeed(0.3)));
```

> **Note:**  
> The `Commands.sequence`, `Commands.parallel`, and similar factory methods are convenience wrappers. You can also create command groups directly using classes like `SequentialCommandGroup` or `ParallelCommandGroup`, passing your commands as parameters.

**Example using `ParallelCommandGroup`:**

```java
Command parallelGroup = new ParallelCommandGroup(
     new InstantCommand(() -> armSubsystem.setArmSpeed(0.5), armSubsystem),
     new InstantCommand(() -> intakeSubsystem.deploy(), intakeSubsystem)
);
```

## 6. Setting Up RobotContainer

### The RobotContainer Class

RobotContainer is where most of the setup for your command-based robot will take place. In this class, you will define your robot's subsystems and commands, bind those commands to triggering events (such as buttons), and specify which command you will run in your autonomous routine.

### Basic RobotContainer Structure

```java
public class RobotContainer {
     // Subsystems
     private final ArmSubsystem armSubsystem = new ArmSubsystem();
     private final DriveSubsystem driveSubsystem = new DriveSubsystem();
     private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
     
     // Controllers
     private final CommandXboxController driverController = 
          new CommandXboxController(Constants.OIConstants.DRIVER_CONTROLLER_PORT);
     
     public RobotContainer() {
          // Configure default commands
          configureDefaultCommands();
          
          // Configure button bindings
          configureBindings();
     }
     
     private void configureDefaultCommands() {
          // Set default command for drivetrain
          driveSubsystem.setDefaultCommand(
                run(() -> driveSubsystem.arcadeDrive(
                     -driverController.getLeftY(),
                     -driverController.getRightX()
                )).withName("Default Drive")
          );
          // Set default command for arm (e.g., hold position)
          armSubsystem.setDefaultCommand(
                armSubsystem.holdArmCommand()
          );
     }
     
     private void configureBindings() {
          // Button bindings go here
     }
     
     public Command getAutonomousCommand() {
          // Return autonomous command
          return null;
     }
}
```

### Subsystem Declaration

Notice that subsystems are declared as private fields in RobotContainer. This is in stark contrast to the previous incarnation of the command-based framework, but is much more-aligned with agreed-upon object-oriented best-practices.

### Reference material
- [Organizing Robot Code](https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based-project.html)

## 7. Binding Commands to Triggers

### Understanding Triggers

Apart from autonomous commands, which are scheduled at the start of the autonomous period, and default commands, which are automatically scheduled whenever their subsystem is not currently in-use, the most common way to run a command is by binding it to a triggering event, such as a button being pressed by a human operator.

### Button Bindings

Command binding is done through the Trigger class. The command-based HID classes contain factory methods returning a Trigger for a given button.

```java
private void configureBindings() {
     // Button press bindings
     driverController.a().onTrue(
          runOnce(() -> intakeSubsystem.deploy())
     );
     
     // While button is held
     driverController.b().whileTrue(
          run(() -> armSubsystem.setArmSpeed(0.5))
     );
     
     // When button is released
     driverController.x().onFalse(
          runOnce(() -> armSubsystem.setArmSpeed(0))
     );
     
     // Toggle command
     driverController.y().toggleOnTrue(
          run(() -> armSubsystem.setArmSpeed(-0.5))
     );
}
```

### Trigger Conditions

You can also bind commands to custom conditions:

```java
// Bind to sensor readings
new Trigger(() -> armSubsystem.getArmPosition() > 100)
     .onTrue(runOnce(() -> armSubsystem.setArmSpeed(0)));

// Bind to subsystem conditions
new Trigger(armSubsystem::isAtTargetPosition)
     .onTrue(runOnce(() -> System.out.println("Arm at target!")));
```

### Reference material
- [Binding Commands to Triggers](https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html)

## 8. The Command Scheduler

### How the Scheduler Works

The CommandScheduler is the class responsible for actually running commands. Each iteration (ordinarily once per 20ms), the scheduler polls all registered buttons, schedules commands for execution, and runs all scheduled commands.

### Scheduler Process

The scheduler follows this process every 20ms:

1. **Poll Triggers**: Check all registered triggers for state changes
2. **Schedule Commands**: Start new commands based on trigger conditions
3. **Run Commands**: Call execute() on all active commands
4. **Check Completion**: Call isFinished() and end() as needed
5. **Resource Management**: Ensure subsystem requirements are respected

### Resource Management

Multiple commands can run concurrently, as long as they do not require the same resources on the robot. Resource management is handled on a per-subsystem basis: commands specify which subsystems they interact with, and the scheduler will ensure that no more more than one command requiring a given subsystem is scheduled at a time.

### Reference material
- [Command Scheduler](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-scheduler.html)

## 9. Constants and Organization

### Constants Class

Organize all your robot's constants in one place in a separate class from your subsystem. This declutters your subsystem and command program.

> **Note:**  
> Some teams choose to create a separate `Constants` class for each subsystem (e.g., `DriveConstants`, `ArmConstants`, `IntakeConstants`) instead of grouping all constants in a single file. This can help keep constants closely associated with their respective subsystems and improve maintainability in larger projects.

```java
public final class Constants {
     public static final class DriveConstants {
          public static final int LEFT_MOTOR_PORT = 0;
          public static final int RIGHT_MOTOR_PORT = 1;
          public static final double DRIVE_SPEED = 0.8;
          public static final double TURN_SPEED = 0.6;
     }
     
     public static final class ArmConstants {
          public static final int ARM_MOTOR_PORT = 2;
          public static final double ARM_UP_SPEED = 0.5;
          public static final double ARM_DOWN_SPEED = -0.5;
     }
     
     public static final class IntakeConstants {
          public static final int INTAKE_MOTOR_PORT = 3;
          public static final double INTAKE_SPEED = 0.75;
     }
     
     public static final class OIConstants {
          public static final int DRIVER_CONTROLLER_PORT = 0;
          public static final int OPERATOR_CONTROLLER_PORT = 1;
     }
}
```

### Project Organization Tips

1. **Group Related Commands**: Create subfolders in the commands directory
2. **Use Descriptive Names**: Make your intent clear in class and method names
3. **Keep Methods Small**: Break complex operations into smaller, testable pieces
4. **Comment Your Code**: Explain the "why" behind your decisions

## 10. Testing and Debugging

### Testing Individual Components

1. **Test Subsystems**: Use the subsystem's periodic() method for monitoring with Smartdashboard or Advantagekit Logging.
2. **Test Commands**: Run commands individually to verify behavior. Use the printline technique shown below for verifying command functionality

### Common Debugging Techniques

```java
// Add logging to commands
@Override
public void initialize() {
     System.out.println("Command started: " + getName());
}

// Monitor subsystem state
@Override
public void periodic() {
     SmartDashboard.putNumber("Arm Speed", armMotor.get());
     SmartDashboard.putNumber("Arm Position", armEncoder.getDistance());
}
```

## 11. Best Practices Summary

### Design Principles

1. **Single Responsibility**: Each class should have one clear purpose
2. **Encapsulation**: Hide implementation details within subsystems
3. **Dependency Injection**: Pass subsystems to commands through constructors
4. **Declarative Programming**: Set up behavior once, let the framework handle execution

### Code Organization

1. **Use the Standard Structure**: Follow the WPILib template organization
2. **Keep Constants Organized**: Group related constants together
3. **Write Factory Methods**: Create reusable command factories in subsystems
4. **Use Descriptive Names**: Make your code self-documenting

### Example: Real-World Project Structure (PhantomCatz 2025)

Below is an outline of the `frc/robot` directory from the referenced project:

```
frc/robot/
├── Constants.java
├── Main.java
├── Robot.java
├── RobotContainer.java
├── commands/
│   ├── auto/
│   ├── drive/
│   ├── arm/
│   ├── intake/
│   └── ... (other command groups)
├── subsystems/
│   ├── DriveSubsystem.java
│   ├── ArmSubsystem.java
│   ├── IntakeSubsystem.java
│   └── ... (other subsystems)
└── util/
     └── ... (utility classes)
```

**Key Points:**
- Commands are grouped by function (e.g., `auto`, `drive`, `arm`, `intake`).
- Subsystems are in their own folder.
- Utility code is separated into `lib/util` and `util` folders.
- Core files (`Robot.java`, `RobotContainer.java`, `Constants.java`) are at the root.

This structure helps keep code organized, scalable, and easy to navigate as the robot project grows.

**Reference:**  
You can view a real-world example of a command-based robot project on GitHub:  
[PhantomCatz 2025 Robot Code](https://github.com/PhantomCatz/RobotCode2025-Reefscape/tree/main/SeasonCode2025/src/main/java/frc/robot)

### Testing Strategy

1. **Test Incrementally**: Build and test one component at a time
2. **Use Default Commands**: Set up safe default behaviors
3. **Handle Edge Cases**: Consider what happens when things go wrong
4. **Log Important Events**: Use console output or SmartDashboard for debugging

## 12. Next Steps

Once you have a basic command-based robot working, consider exploring:

- **Advanced Command Compositions**: More complex parallel and sequential operations
- **PID Control**: Closed-loop control for precise movements
- **Path Following**: Autonomous navigation using trajectory generation
- **State Machines**: Managing complex robot behaviors
- **Custom Triggers**: Creating sophisticated activation conditions

The command-based framework is powerful and flexible. Start with simple commands and subsystems, then gradually add complexity as you become more comfortable with the paradigm.

