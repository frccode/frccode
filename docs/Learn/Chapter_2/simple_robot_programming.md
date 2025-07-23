# An Intro Through Timed Robot
At this point in the guide, you should have your VSCode and WPILib installed, as well as a fundamental understanding of the Java programming language. 

!!! important
    TS NOT DONE

This module will introduce you to many beginner concepts that will be used throughout all of FRC programming using `TimedRobot` as an example. While other robot architectures are more popular and often more powerful, `TimedRobot` serves as an excellent stepping stone to introduce the fundamentals. 

This training will overview/simplify explantations from [WPILib's official Zero-to-Robot guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-1/index.html). 

## Introduction to the TimedRobot Template

WPILib provides the `TimedRobot` base class to simplify handling robot periodic actions. A TimedRobot automatically calls key methods at regular intervals (default every 20 milliseconds), allowing you to focus on writing logic rather than managing timing.

### The `TimedRobot` Class

When you create a new robot project in VSCode with WPILib, the generated `Robot.java` will extend `TimedRobot`. This class includes several lifecycle methods that you can implement based on a time loop:

```java

```



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
