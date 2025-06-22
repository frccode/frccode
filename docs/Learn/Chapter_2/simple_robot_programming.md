# An Intro Through Timed Robot
At this point in the guide, you should have your VSCode and WPILib installed, as well as a fundamental understanding of the Java programming language. 

!!! important
    TS NOT DONE

This module will introduce you to many beginner concepts that will be used throughout all of FRC programming using `TimedRobot` as an example. While other robot architectures are more popular and often more powerful, `TimedRobot` serves as an excellent stepping stone to introduce the fundamentals. 

## Introduction to the TimedRobot Template

WPILib provides the `TimedRobot` base class to simplify handling robot periodic actions. A TimedRobot automatically calls key methods at regular intervals (default every 20 milliseconds), allowing you to focus on writing logic rather than managing timing.

### The `TimedRobot` Class

When you create a new robot project in VSCode with WPILib, the generated `Robot.java` will extend `TimedRobot`. This class includes several lifecycle methods that you can implement:

- `robotInit()`: Runs once when the robot code first starts. Use this to initialize sensors, motors, cameras, and other hardware.
- `autonomousInit()`: Called once each time the robot enters Autonomous mode. Reset relevant timers, encoders, or state variables here.
- `autonomousPeriodic()`: Called periodically (every 20ms by default) during Autonomous mode. Place autonomous routines and sensor checks here.
- `teleopInit()`: Called once each time the robot enters Teleoperated (Teleop) mode. Prepare any state for driver control.
- `teleopPeriodic()`: Called periodically during Teleop mode. Read joystick inputs, update motor outputs, and run control loops here.
- `disabledInit()` and `disabledPeriodic()`: Called when the robot is disabled. Useful for stopping motors or updating dashboards safely.

### Basic Project Structure

Here’s an example skeleton of a `Robot.java` file:

```java
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

    @Override
    public void robotInit() {
        // Initialize hardware and state
    }

    @Override
    public void autonomousInit() {
        // Reset sensors or timers before autonomous
    }

    @Override
    public void autonomousPeriodic() {
        // Autonomous control logic
    }

    @Override
    public void teleopInit() {
        // Any setup before teleop
    }

    @Override
    public void teleopPeriodic() {
        // Driver control code
    }

    @Override
    public void disabledInit() {
        // Actions on disable
    }

    @Override
    public void disabledPeriodic() {
        // Periodic actions when disabled
    }
}
```
