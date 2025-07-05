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

## 4. AdvantageKit: Advanced Logging Framework

### What Makes AdvantageKit Different?

AdvantageKit takes a different approach. Instead of logging a limited set of values from the user code, AdvantageKit records all of the data flowing into the robot code. Every sensor value, button press, and much more is logged every loop cycle. After a match, these values can be replayed to the robot code in a simulator.

**Key Benefits:**
- **Complete Replay Capability**: Replay exactly what happened during a match
- **Post-Match Analysis**: Add logging after the fact to debug issues
- **Input/Output Separation**: Clear distinction between sensor inputs and calculated outputs

### AdvantageKit Project Structure

AdvantageKit works best with an IO layer pattern:

```
src/main/java/frc/robot/
├── Robot.java
├── RobotContainer.java
├── Constants.java
├── subsystems/
│   ├── drive/
│   │   ├── DriveSubsystem.java
│   │   ├── DriveIO.java          // Interface
│   │   ├── DriveIOSparkMax.java  // Real hardware
│   │   └── DriveIOSim.java       // Simulation
│   └── arm/
│       ├── ArmSubsystem.java
│       ├── ArmIO.java
│       ├── ArmIOTalonFX.java
│       └── ArmIOSim.java
└── commands/
```

### Creating an IO Interface

The IO interface defines what data flows in and out:

```java
public interface DriveIO {
    public static class DriveIOInputs implements LoggableInputs {
        public double leftPositionRad = 0.0;
        public double leftVelocityRadPerSec = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;
        
        public double rightPositionRad = 0.0;
        public double rightVelocityRadPerSec = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;
        
        public double gyroYawRad = 0.0;
        
        // These methods are auto-generated by AdvantageKit
        public void toLog(LogTable table) {
            // Auto-generated logging code
        }
        
        public void fromLog(LogTable table) {
            // Auto-generated replay code
        }
    }
    
    // Methods for updating inputs and setting outputs
    public default void updateInputs(DriveIOInputs inputs) {}
    public default void setVoltage(double leftVolts, double rightVolts) {}
    public default void setBrakeMode(boolean enable) {}
}
```

### Implementing Real Hardware IO

```java
public class DriveIOSparkMax implements DriveIO {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final AHRS gyro;
    
    public DriveIOSparkMax() {
        leftMotor = new CANSparkMax(Constants.DriveConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.DriveConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        gyro = new AHRS(Port.kMXP);
        
        // Configure motors
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
    }
    
    @Override
    public void updateInputs(DriveIOInputs inputs) {
        // Read all sensor data and populate inputs
        inputs.leftPositionRad = Units.rotationsToRadians(leftEncoder.getPosition());
        inputs.leftVelocityRadPerSec = Units.rpmToRadiansPerSecond(leftEncoder.getVelocity());
        inputs.leftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
        inputs.leftCurrentAmps = leftMotor.getOutputCurrent();
        
        inputs.rightPositionRad = Units.rotationsToRadians(rightEncoder.getPosition());
        inputs.rightVelocityRadPerSec = Units.rpmToRadiansPerSecond(rightEncoder.getVelocity());
        inputs.rightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
        inputs.rightCurrentAmps = rightMotor.getOutputCurrent();
        
        inputs.gyroYawRad = Units.degreesToRadians(gyro.getYaw());
    }
    
    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftMotor.setVoltage(leftVolts);
        rightMotor.setVoltage(rightVolts);
    }
    
    @Override
    public void setBrakeMode(boolean enable) {
        IdleMode mode = enable ? IdleMode.kBrake : IdleMode.kCoast;
        leftMotor.setIdleMode(mode);
        rightMotor.setIdleMode(mode);
    }
}
```

### Using IO in Subsystems

```java
public class DriveSubsystem extends SubsystemBase {
    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    
    public DriveSubsystem(DriveIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        // Update inputs from hardware
        io.updateInputs(inputs);
        
        // Process inputs with AdvantageKit (logs on real robot, replays in sim)
        Logger.processInputs("Drive", inputs);
        
        // Your control logic here using inputs.leftVelocityRadPerSec, etc.
        // This logic will work identically during replay!
    }
    
    public void driveArcade(double forward, double turn) {
        // Calculate motor voltages
        double leftVolts = (forward + turn) * 12.0;
        double rightVolts = (forward - turn) * 12.0;
        
        // Send commands to hardware
        io.setVoltage(leftVolts, rightVolts);
        
        // Log outputs for analysis
        Logger.recordOutput("Drive/LeftSetpointVolts", leftVolts);
        Logger.recordOutput("Drive/RightSetpointVolts", rightVolts);
    }
}
```

### Setting Up AdvantageKit Logging

In your Robot.java:

```java
public class Robot extends LoggedRobot {
    @Override
    public void robotInit() {
        // Start AdvantageKit logging
        Logger.recordMetadata("ProjectName", "MyRobot2025");
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("BuildDirty", Byte.toString(BuildConstants.DIRTY));
        
        // Set up data receivers
        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to file
            Logger.addDataReceiver(new NT4Publisher()); // Publish to NetworkTables
        } else {
            setUseTiming(false); // Run as fast as possible in sim
            String logPath = LogFileUtil.findReplayLog(); // Prompt user for replay log
            Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.addDataReceiver(new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to new log
        }
        
        Logger.start(); // Start logging
        
        // Initialize robot
        robotContainer = new RobotContainer();
    }
}
```

## 5. Recording Output Data

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

## 6. Best Practices for Logging

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

## 7. Viewing and Analyzing Logs

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

## 8. Common Pitfalls and Solutions

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

## 9. Getting Started Checklist

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

### AdvantageKit Setup
- [ ] Install AdvantageKit in your project
- [ ] Create IO interfaces for subsystems
- [ ] Implement real hardware and simulation IO classes
- [ ] Set up Logger in `robotInit()` with appropriate receivers
- [ ] Test replay functionality with a recorded log

### Best Practices
- [ ] Use meaningful, hierarchical names for log entries
- [ ] Monitor memory usage and performance
- [ ] Log state changes and important events
- [ ] Set up AdvantageScope for log analysis
- [ ] Create a process for reviewing logs after matches

## 10. Next Steps

Once you have basic logging working:

- **Advanced Analysis**: Learn to use AdvantageScope's advanced features
- **Automated Testing**: Use logged data to validate code changes
- **Performance Optimization**: Analyze loop timing and optimize critical paths
- **Competition Strategy**: Review match logs to improve strategy and driving
- **System Monitoring**: Set up alerts for unusual sensor readings or performance issues

Remember: Logging is your safety net. The data you don't log today might be exactly what you need to debug a problem tomorrow!