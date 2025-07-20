# Robot Logging and Telemetry Guide

> **Note:** This guide introduces the fundamentals of robot logging and telemetry, covering both WPILib's built-in logging capabilities and AdvantageKit's advanced framework. Start with the basics and gradually work your way to more advanced logging concepts.

## 1. Understanding Robot Logging

### What is Robot Logging?

Robot logging is the process of recording data from your robot during operation. This includes sensor readings, motor commands, calculated values, and system performance metrics. Think of it as creating a "black box" recording of everything your robot does.

### Why Log Data?

**Visibility into Robot Performance:**
- Understand robot characteristics (How fast can the robot go? How long does acceleration take?)
- Monitor system health and performance
- Track sensor accuracy and motor behavior

**Troubleshooting and Debugging:**
- Identify problems after they occur
- Less intrusive than print statements (which can slow down or "hide" timing issues)
- Analyze robot behavior during matches or testing

**Performance Analysis:**
- Compare expected vs. actual behavior
- Optimize control algorithms
- Validate autonomous routines

**Safety Net:**
- Record unexpected behaviors for later analysis
- Verify code changes work as expected

### Important Memory Considerations

⚠️ **Memory Management Warning:**
- More logged values = more memory per record
- High sample rates = faster memory consumption
- Current frameworks don't automatically limit memory usage
- Monitor memory consumption to avoid system crashes

## 2. WPILib Built-In Logging

### Setting Up Basic WPILib Logging

WPILib provides simple built-in logging through the `DataLogManager`:

```java
public class Robot extends TimedRobot {
    @Override
    public void robotInit() {
        // Start data logging to a file on the roboRIO
        DataLogManager.start();
        
        // Optional: Log driver station data and joystick inputs
        DriverStation.startDataLog(DataLogManager.getLog());
    }
}
```

### Manual Logging with DataLogManager

Log specific values using WPILib's logging system:

```java
public class DriveSubsystem extends SubsystemBase {
    private final DoubleLogEntry leftSpeedLog;
    private final DoubleLogEntry rightSpeedLog;
    private final BooleanLogEntry coastModeLog;
    
    public DriveSubsystem() {
        // Create log entries
        DataLog log = DataLogManager.getLog();
        leftSpeedLog = new DoubleLogEntry(log, "/drive/leftSpeed");
        rightSpeedLog = new DoubleLogEntry(log, "/drive/rightSpeed");
        coastModeLog = new BooleanLogEntry(log, "/drive/coastMode");
    }
    
    @Override
    public void periodic() {
        // Log current values
        leftSpeedLog.append(leftMotor.get());
        rightSpeedLog.append(rightMotor.get());
        coastModeLog.append(isCoastMode);
    }
}
```

### SmartDashboard Logging

For real-time viewing and simple logging:

```java
@Override
public void periodic() {
    // Display on dashboard and automatically log
    SmartDashboard.putNumber("Arm Position", armEncoder.getDistance());
    SmartDashboard.putNumber("Arm Speed", armMotor.get());
    SmartDashboard.putBoolean("At Target", isAtTarget());
}
```

## 3. New for 2025: WPILib Annotation Logging

### Quick Start with @Logged Annotation

The easiest way to start logging with WPILib 2025:

```java
@Logged
public class Robot extends TimedRobot {
    private final ArmSubsystem armSubsystem;
    private final DriveSubsystem driveSubsystem;
    
    public Robot() {
        armSubsystem = new ArmSubsystem();
        driveSubsystem = new DriveSubsystem();
        
        // Optional: Save to file on roboRIO
        DataLogManager.start();
        
        // Start annotation logging
        Epilogue.bind(this);
    }
}

@Logged
public class ArmSubsystem extends SubsystemBase {
    private final PWMSparkMax armMotor = new PWMSparkMax(Constants.ArmConstants.ARM_MOTOR_PORT);
    private final Encoder armEncoder = new Encoder(0, 1);
    
    // All public fields and methods will be automatically logged!
    public double getArmPosition() {
        return armEncoder.getDistance();
    }
    
    public double getArmSpeed() {
        return armMotor.get();
    }
}
```

### Customizing Annotation Logging

Control what gets logged and how:

```java
@Logged
public class ArmSubsystem extends SubsystemBase {
    // Log with custom name and importance
    @Logged(name = "Motor Temperature", importance = Logged.Importance.CRITICAL)
    private double motorTemp;
    
    // Exclude from logging
    @NotLogged
    private double internalCalculation;
    
    // Custom logging frequency
    @Logged(name = "Position", importance = Logged.Importance.DEBUG)
    public double getPosition() {
        return armEncoder.getDistance();
    }
}
```

### Configuring Epilogue

Customize the logging behavior:

```java
public Robot() {
    Epilogue.configure(config -> {
        // Only log critical information to reduce bandwidth
        config.minimumImportance = Logged.Importance.CRITICAL;
        
        // Change root path in logs
        config.root = "MyRobot";
        
        // Handle errors differently in simulation
        if (isSimulation()) {
            config.errorHandler = ErrorHandler.crashOnError();
        }
    });
    
    Epilogue.bind(this);
}
```

## 4. Recording Output Data

### Basic Output Logging

Output data consists of any calculated values which could be recreated in the simulator:

```java
// In your subsystem or command
public void periodic() {
    // Log simple values
    Logger.recordOutput("Flywheel/Setpoint", setpointSpeed);
    Logger.recordOutput("Drive/LeftVelocity", leftVelocity);
    Logger.recordOutput("Arm/AtTarget", isAtTarget());
    
    // Log complex objects
    Logger.recordOutput("Drive/Pose", odometryPose);
    Logger.recordOutput("Vision/TargetPose", targetPose);
    Logger.recordOutput("Swerve/States", moduleStates);
}
```

### Automatic Output Logging

AdvantageKit can automatically log outputs using annotations:

```java
public class DriveSubsystem extends SubsystemBase {
    @AutoLogOutput // Automatically logged as "DriveSubsystem/currentPose"
    private Pose2d currentPose = new Pose2d();
    
    @AutoLogOutput(key = "Drive/Velocity") // Custom log key
    public double getVelocity() {
        return (inputs.leftVelocityRadPerSec + inputs.rightVelocityRadPerSec) / 2.0;
    }
    
    @AutoLogOutput(key = "Drive/ModuleStates")
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] { /* ... */ };
    }
}
```

## 5. Best Practices for Logging

### Memory Management

**Monitor Memory Usage:**
- Check `/Logger/QueuedCycles` in logs to monitor data queue
- Reduce logging frequency for non-critical data
- Use importance levels to filter data during competition

**Optimize Data Types:**
- Log primitive types when possible
- Avoid logging large arrays every cycle
- Consider logging rates (some data doesn't need 50Hz)

### Performance Considerations

**Avoid Expensive Operations:**
- Data logging increases CPU load on the roboRIO and can lead to loop overruns. The CPU load is not caused by the logging itself, but from calling expensive methods to read data.
- Cache expensive calculations instead of recalculating for logs
- Be mindful of CAN bus traffic from frequent sensor queries

**Monitor Performance:**
- Use `/LoggedRobot/FullCycleMS` to monitor total loop time
- Keep loop times under 20ms (or your configured period)
- Watch for garbage collection time in `/LoggedRobot/GCTimeMS`

### Organization and Structure

**Use Meaningful Names:**
```java
// Good
Logger.recordOutput("Arm/Position/Shoulder", shoulderAngle);
Logger.recordOutput("Arm/Position/Elbow", elbowAngle);
Logger.recordOutput("Arm/Setpoint/Shoulder", shoulderSetpoint);

// Avoid
Logger.recordOutput("angle1", shoulderAngle);
Logger.recordOutput("pos", elbowAngle);
```

**Group Related Data:**
```java
// Group by subsystem and function
Logger.recordOutput("Drive/Odometry/Pose", pose);
Logger.recordOutput("Drive/Odometry/Velocity", velocity);
Logger.recordOutput("Drive/Control/LeftSetpoint", leftSetpoint);
Logger.recordOutput("Drive/Control/RightSetpoint", rightSetpoint);
```

### Debugging Strategy

**Start Simple:**
1. Begin with basic sensor inputs and motor outputs
2. Add calculated values as needed
3. Include state machine states and important flags

**Use Conditional Logging:**
```java
// Only log debug info when needed
if (DriverStation.isTest()) {
    Logger.recordOutput("Debug/DetailedCalc", expensiveCalculation());
}
```

**Log State Changes:**
```java
public void setState(ArmState newState) {
    if (newState != currentState) {
        Logger.recordOutput("Arm/StateChange", newState.toString());
        currentState = newState;
    }
}
```

## 6. Viewing and Analyzing Logs

### AdvantageScope

AdvantageScope is the recommended tool for viewing logs:
- Works with both WPILib and AdvantageKit logs
- Real-time data viewing
- Advanced analysis tools
- 3D robot visualization

### Common Analysis Workflows

**Performance Analysis:**
1. Check loop timing in `/LoggedRobot/FullCycleMS`
2. Verify sensor data makes sense
3. Compare setpoints vs. actual values
4. Look for unexpected behavior patterns

**Debugging Issues:**
1. Find when the problem occurred
2. Check all relevant sensor inputs at that time
3. Verify control logic calculations
4. Look for error patterns or state changes

**Match Analysis:**
1. Review autonomous performance
2. Check for brownouts or connection issues
3. Analyze driver control patterns
4. Verify robot behavior matches strategy

## 7. Common Pitfalls and Solutions

### Print Statements vs. Logging

**Problem:** Using `System.out.println()` for debugging
```java
// DON'T DO THIS
@Override
public void periodic() {
    System.out.println("Arm position: " + getPosition());
    System.out.println("Arm speed: " + getSpeed());
}
```

**Solution:** Use proper logging
```java
// DO THIS INSTEAD
@Override
public void periodic() {
    Logger.recordOutput("Arm/Position", getPosition());
    Logger.recordOutput("Arm/Speed", getSpeed());
}
```

**Why:** Print statements can slow down your code and "hide" timing issues.

### Logging Too Much Data

**Problem:** Logging everything at maximum frequency
```java
// This can overwhelm the system
@Override
public void periodic() {
    Logger.recordOutput("Debug/Value1", calc1());
    Logger.recordOutput("Debug/Value2", calc2());
    Logger.recordOutput("Debug/Value3", calc3());
    // ... 20 more values
}
```

**Solution:** Use importance levels and selective logging
```java
// Log critical data always, debug data only when needed
@Override
public void periodic() {
    Logger.recordOutput("Arm/Position", getPosition()); // Always log
    
    if (DriverStation.isTest()) {
        Logger.recordOutput("Debug/InternalCalc", expensiveDebugCalc());
    }
}
```

### Not Logging State Changes

**Problem:** Missing important events
```java
public void setState(State newState) {
    currentState = newState;
    // No logging of the change!
}
```

**Solution:** Always log state transitions
```java
public void setState(State newState) {
    if (newState != currentState) {
        Logger.recordOutput("StateMachine/PreviousState", currentState.toString());
        Logger.recordOutput("StateMachine/NewState", newState.toString());
        Logger.recordOutput("StateMachine/Timestamp", Timer.getFPGATimestamp());
        currentState = newState;
    }
}
```

## 8. Getting Started Checklist

### Basic WPILib Logging
- [ ] Add `DataLogManager.start()` to `robotInit()`
- [ ] Add key sensor readings to subsystem `periodic()` methods
- [ ] Use SmartDashboard for values you want to see in real-time
- [ ] Test that log files are created on the roboRIO

### WPILib Annotation Logging (2025)
- [ ] Add `@Logged` to your Robot class and subsystems
- [ ] Call `Epilogue.bind(this)` in Robot constructor
- [ ] Configure Epilogue settings for your needs
- [ ] Test automatic logging of fields and methods

### Best Practices
- [ ] Use meaningful, hierarchical names for log entries
- [ ] Monitor memory usage and performance
- [ ] Log state changes and important events
- [ ] Set up AdvantageScope for log analysis
- [ ] Create a process for reviewing logs after matches

## 9. Next Steps

Once you have basic logging working:

- **Advanced Analysis**: Learn to use AdvantageScope's advanced features
- **Automated Testing**: Use logged data to validate code changes
- **Performance Optimization**: Analyze loop timing and optimize critical paths
- **Competition Strategy**: Review match logs to improve strategy and driving
- **System Monitoring**: Set up alerts for unusual sensor readings or performance issues
