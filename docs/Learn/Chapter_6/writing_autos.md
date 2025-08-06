# **FRC Autonomous Programming - Technical Implementation**

## Overview

This guide covers the technical implementation of autonomous routines for FRC robots, including control systems, tuning processes, state machines, and testing methodologies.

> **Note:** This guide assumes you have completed the strategic planning process covered in [FRC Autonomous Programming - Strategy and Planning](../autonomous_strategy.md).

**What you'll build:** A complete technical autonomous system with precise control, robust state management, and reliable execution.

---

## 1. Architecture and Hardware Setup

### Robot Design/Construction for Autonomous Success
Certain design/fabrication choices significantly impact autonomous capability. Teams have different philosophies concerning whether certain issues or capabilities fall under a software, electrical or mechanical jurisdiction. That being said, not everything can be "fixed in software" and it's up to the software developer at hand to make that determination.

Below are general guidelines to help you ensure robust autonomous performance on everything "non-software" related. Ensure you work with your mechanical, electrical and CAD subunit of your team to work out the following:

**Enabling Design Features:**

Design mechanisms to support key autonomous tasks. For example, in 2025's "Reefscape," teams chose between ground or station intakes. Ground intakes improved autonomous performance but added design complexity. Teams had to choose which was more important to reaching their competition goals.

```java
// Tank intake example - allows pickup while driving
public class IntakeSubsystem extends SubsystemBase {
    public void deployAndRun() {
        deploy();
        startRollers();
        // Robot can drive while this runs
    }
}
```

**Vision-Friendly Camera Placement:**

Vision supplemented pose estimation will ensure accurate robot movements, especially as teams look to score more game pieces with faster drivebases. Ensure:

- Clear sight lines to commonly used AprilTags (ie. At scoring and loading zones)
- Place cameras at different corners or heights on the robot to maximize AprilTag visibility and minimize occlusion from mechanisms or game pieces.
- Protected from game piece interference and damage
- Multiple cameras for more points of triangulation to estimate your robot's position.

**Fast, Consistent Mechanisms:**
- Minimize/optimize areas of handoff and degrees of freedom
- Repeatable positioning without mechanism backlash and robust sensor feedback.
- Extending mechanisms past the frame increases torque and risk of damage. Ensure mechanisms (elevators, pivot arms, etc.) are well-supported for accuracy and durability.

**Reliable Wiring for Autonomous Consistency:**

Autonomous performance also depends on robust electrical connections. Poor wiring can cause intermittent failures, sensor dropouts, or unexpected resets than can make troubleshooting electrical vs programming errors difficult.

Best Practices:
- Securing all connectors with cable ties or heat shrink to prevent vibration-induced disconnects.
- Label all connectors for quick troubleshooting.
- Perform a "wiggle test"—gently move wires while the robot is powered to check for intermittent faults.

Example: CAN Bus Reliability Checklist:
- Terminate CAN bus at both ends with 120Ω resistors.
- Keep CAN wiring as short and direct as possible.
- Use twisted pair wire for CAN to reduce noise.
- Monitor CAN utilization and error counts in telemetry.

---

## 2. Control System Implementation

Effective autonomous routines depend on a well-tuned software foundation, with the "first layer" starting with reliable motor control. 

Case in point, many teams have struggled with trajectory control for swerve because they only focused on tuning their pathplanning tool's pid rather than building their control system from the ground up ensuring the motors themselves were tuned properly.

Here are the layers that build up the foundation of a well constructed autonomous control system.

### Layer 1: Motor Control

The foundation of autonomous control starts with precise motor control. For a deeper dive into PID control fundamentals, see [Basic PID Control](../PID.md). However, here are some motor configurations to take note of that teams have used to ensure consistent auto performance.

**TalonFX (Phoenix 6) Voltage Compensation:**  
Ensure consistent motor output regardless of battery voltage.  
```java
// Set voltage compensation saturation to 12V and enable it
talonFX.getConfigurator().apply(new VoltageCompensationConfigs().withSaturation(12.0));
talonFX.setControl(new VoltageOut(0)); // Use voltage control mode if needed
```

**Motion Profiling Setup:**  
For more on motion profiling, see [Simple Motion Profiling](../simple_profiling.md).  
```java
// Configure slot gains for velocity/position control
Slot0Configs slot0 = new Slot0Configs();
slot0.kP = 0.1;
slot0.kI = 0.001;
slot0.kD = 0.01;
slot0.kV = 0.05; // Feedforward for velocity
talonFX.getConfigurator().apply(slot0);

// Motion Magic (Motion Profile) configuration
MotionMagicConfigs mm = new MotionMagicConfigs();
mm.MotionMagicCruiseVelocity = 15000; // sensor units per second
mm.MotionMagicAcceleration = 6000;    // sensor units per second^2
talonFX.getConfigurator().apply(mm);
```

**Current Limiting:**  
Protect against brownouts and ensure consistent acceleration.  
```java
CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
currentLimits.SupplyCurrentLimitEnable = true;
currentLimits.SupplyCurrentLimit = 35.0;
currentLimits.SupplyCurrentThreshold = 40.0;
currentLimits.SupplyTimeThreshold = 0.5;
talonFX.getConfigurator().apply(currentLimits);
```

**Neutral Deadband Adjustment:**  
Reduce deadband for more precise low-speed control.  
```java
talonFX.getConfigurator().apply(new MotorOutputConfigs().withNeutralDeadband(0.01));
```

**Brake/Coast Mode Selection:**  
Use brake mode for precise stopping in autonomous.  
```java
talonFX.setNeutralMode(NeutralModeValue.Brake);
```

**Open Loop Ramp Rate:**  
Smooth acceleration to prevent wheel slip.  
```java
talonFX.getConfigurator().apply(new OpenLoopRampsConfigs().withOpenLoopRampPeriod(0.2));
```

**Closed Loop Ramp Rate:**  
Prevents sudden jumps in velocity setpoints.  
```java
talonFX.getConfigurator().apply(new ClosedLoopRampsConfigs().withClosedLoopRampPeriod(0.1));
```

### Layer 2: Localization and Sensors

#### Wheel Odometry Calibration

Wheel odometry calibration ensures your robot's position estimates from encoders are accurate. It corrects for errors like wheel size differences, wheelbase mismeasurements, and encoder inaccuracies. Proper calibration reduces drift, improves path following, and should be repeated after wheel changes or significant use. Below is a simple calculation that determines the "odometry wheel radius" for driving.

```java
// Critical: Determine actual wheel radius on carpet
public void calibrateWheelRadius() {
    // Drive known distance at low acceleration
    double commandedDistance = 3.0; // meters
    double actualDistance = measureActualDistance();
    double reportedDistance = getOdometryDistance();
    
    double correctionFactor = actualDistance / reportedDistance;
    wheelRadius *= correctionFactor;
}
```


**Vision System Integration:**

Wheel odometry alone can drift over time due to wheel slip, uneven surfaces, or mechanical inaccuracies, leading to increasing errors in the robot's estimated position. 

Integrating vision allows the robot to periodically correct its pose estimate using absolute field references. This fusion of sensor data combines the short-term accuracy and responsiveness of wheel odometry with the long-term reliability of vision-based localization, resulting in more precise autonomous movement, better path following, and improved scoring consistency, especially in complex or multi-piece autonomous routines.

```java
public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    
    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();
        
        if (result.hasTargets()) {
            Optional<EstimatedRobotPose> pose = poseEstimator.update(result);
            if (pose.isPresent()) {
                // Add vision measurement to pose estimator
                driveSubsystem.addVisionMeasurement(
                    pose.get().estimatedPose.toPose2d(),
                    pose.get().timestampSeconds
                );
            }
        }
    }
}
```

### Layer 3: Path Following
```java
public class AutoDriveCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final PIDController xController;
    private final PIDController yController; 
    private final PIDController rotationController;
    private final Trajectory trajectory;
    
    public AutoDriveCommand(DriveSubsystem drive, Trajectory trajectory) {
        this.driveSubsystem = drive;
        this.trajectory = trajectory;
        
        // Tuned PID controllers for path following
        xController = new PIDController(1.0, 0.0, 0.0);
        yController = new PIDController(1.0, 0.0, 0.0);
        rotationController = new PIDController(0.5, 0.0, 0.0);
        
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void execute() {
        Trajectory.State goal = trajectory.sample(timer.get());
        Pose2d currentPose = driveSubsystem.getPose();
        
        // Calculate velocity commands
        double xVel = xController.calculate(currentPose.getX(), goal.poseMeters.getX());
        double yVel = yController.calculate(currentPose.getY(), goal.poseMeters.getY());
        double rotVel = rotationController.calculate(
            currentPose.getRotation().getRadians(), 
            goal.poseMeters.getRotation().getRadians()
        );
        
        driveSubsystem.drive(new ChassisSpeeds(xVel, yVel, rotVel));
    }
}
```

---

## 3. Systematic Tuning Process

Tune each layer sequentially to ensure a solid foundation—if earlier layers are not properly tuned, later layers will struggle to compensate, leading to unreliable autonomous performance.

### Step 1: Motor Control Tuning
Using visual feedback for optimal performance:

### Step 2: Localization Tuning

### Step 3: Path Following Tuning

---

## 4. State Machine Implementation

### Robust Robot State Management
```java
public enum SuperstructureState {
    IDLE,
    INTAKE_DEPLOY,
    INTAKE_RUNNING,
    PREPARE_SCORE,
    SCORING,
    STOWED
}

public class SuperstructureSubsystem extends SubsystemBase {
    private SuperstructureState currentState = SuperstructureState.IDLE;
    private SuperstructureState nextState = SuperstructureState.IDLE;
    
    @Override
    public void periodic() {
        switch (currentState) {
            case IDLE:
                // Safe positions, ready for commands
                setElevatorPosition(0);
                setArmPosition(STOWED_ANGLE);
                
                if (requestIntake) {
                    nextState = SuperstructureState.INTAKE_DEPLOY;
                }
                break;
                
            case INTAKE_DEPLOY:
                // Deploy intake mechanism
                setIntakePosition(DEPLOYED);
                
                if (intakeDeployed()) {
                    nextState = SuperstructureState.INTAKE_RUNNING;
                }
                break;
                
            case INTAKE_RUNNING:
                // Run intake rollers, wait for game piece
                runIntakeRollers();
                
                if (gamePieceDetected()) {
                    nextState = SuperstructureState.PREPARE_SCORE;
                } else if (!requestIntake) {
                    nextState = SuperstructureState.IDLE;
                }
                break;
                
            case PREPARE_SCORE:
                // Move to scoring position
                setElevatorPosition(scoreHeight);
                setArmPosition(SCORE_ANGLE);
                
                if (readyToScore()) {
                    nextState = SuperstructureState.SCORING;
                }
                break;
                
            case SCORING:
                // Eject game piece
                ejectGamePiece();
                
                if (scoreComplete()) {
                    nextState = SuperstructureState.STOWED;
                }
                break;
        }
        
        currentState = nextState;
    }
}
```

---

## 5. Autonomous Routine Scripting

### Command-Based Autonomous Structure (Inline Factory Implementation)

While there are many different command based formats for constructing autonomous commands based on each team's individual robot code structure preferences, this is the simplest implementation.

Store these autonomous command and others like it within your "RobotContainer" Class. This makes it easy for you to pass in the subsystems as parameters needed to run the autonomous program.
```java
// Inline SequentialCommandGroup factory for Three Piece Auto
public Command getThreePieceAuto(
    DriveSubsystem drive, 
    SuperstructureSubsystem superstructure,
    IntakeSubsystem intake
) {
    return new SequentialCommandGroup(
        // Score preloaded game piece
        new ParallelCommandGroup(
            new ScoreHigh(superstructure),
            new PrepareForIntake(intake)
        ),

        // Drive to first game piece while deploying intake
        new ParallelDeadlineGroup(
            new FollowTrajectory(drive, "ToFirstPiece"),
            new IntakeGamePiece(intake)
        ),

        // Return and score second piece  
        new ParallelCommandGroup(
            new FollowTrajectory(drive, "FirstPieceReturn"),
            new PrepareToScore(superstructure)
        ),
        new ScoreMid(superstructure),

        // Get third piece
        new ParallelDeadlineGroup(
            new FollowTrajectory(drive, "ToSecondPiece"), 
            new IntakeGamePiece(intake)
        ),

        // Final score and positioning
        new ParallelCommandGroup(
            new FollowTrajectory(drive, "FinalScore"),
            new PrepareToScore(superstructure)
        ),
        new ScoreHigh(superstructure),

        // End in good position for teleop
        new FollowTrajectory(drive, "FinalPosition")
    );
}
```

### Robot Container Integration

```java
// In your RobotContainer class
public class RobotContainer {
    private final DriveSubsystem drive = new DriveSubsystem();
    private final SuperstructureSubsystem superstructure = new SuperstructureSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();

    // Inline SequentialCommandGroup factory for PathPlanner-based auto
    public Command getPathPlannerAuto() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> drive.resetOdometry()),
            new ParallelCommandGroup(
                new StartIntake(intake),
                new PrepareScore(superstructure)
            ),
            // Follow the generated path using PathPlanner
            AutoBuilder.followPath(
                PathPlannerPath.fromPathFile("ThreePiecePath")
            ),
            new StopIntake(intake),
            new ScoreHigh(superstructure)
        );
    }

    // Auto Factories -> See above for more info
}
```

---

## 6. Testing and Validation

### Layer-by-Layer Testing
Test each system independently before integration:

**Motor Control Validation:**
```java
// Verify setpoint tracking
public void testMotorControl() {
    // Plot commanded vs actual position/velocity
    // Check for voltage compensation effectiveness
    // Validate across different battery levels
}
```

**Localization Testing:**
```java  
// Odometry accuracy test
public void testOdometry() {
    // Drive known distance, measure error
    // Should be <2% error over 5+ meters
}

// Vision system test  
public void testVision() {
    // Verify pose updates at various distances
    // Check for consistency across lighting conditions
}
```

**Integration Testing:**
```java
// Full autonomous testing
public void testAutonomous() {
    // Run complete routines repeatedly
    // Test with different starting positions
    // Validate timing assumptions
    // Test failure recovery (missed game pieces, etc.)
}
```

### Field Testing Protocol
When possible, test on competition-like conditions:

```java
// Competition simulation
public class AutonomousTest extends CommandBase {
    private final List<Command> testRoutines;
    private int currentTest = 0;
    
    @Override
    public void execute() {
        if (currentTest < testRoutines.size()) {
            Command routine = testRoutines.get(currentTest);
            routine.schedule();
            
            // Log performance metrics
            logTestResults(routine, currentTest);
            currentTest++;
        }
    }
}
```

---

## 7. Essential Debugging Tips

### Comprehensive Telemetry
```java
@Override
public void periodic() {
    // Pose and localization
    SmartDashboard.putString("Robot Pose", getPose().toString());
    SmartDashboard.putNumber("Vision Targets", getVisibleTargets());
    
    // Path following
    SmartDashboard.putNumber("Path Error X", getPathErrorX());
    SmartDashboard.putNumber("Path Error Y", getPathErrorY()); 
    SmartDashboard.putNumber("Path Error Rot", getPathErrorRotation());
    
    // State machine
    SmartDashboard.putString("Superstructure State", getCurrentState().toString());
    SmartDashboard.putBoolean("At Target", atTarget());
    
    // Performance monitoring
    SmartDashboard.putNumber("Loop Time", getLoopTime());
    SmartDashboard.putNumber("Battery Voltage", getBatteryVoltage());
}
```

### Using AdvantageKit for Analysis
```java
import org.littletonrobotics.junction.Logger;

@Override  
public void periodic() {
    Logger.recordOutput("Drive/Pose", getPose());
    Logger.recordOutput("Drive/Velocity", getChassisSpeeds());
    Logger.recordOutput("Drive/PathTarget", getPathTarget());
    Logger.recordOutput("Vision/Targets", getVisionTargets());
    Logger.recordOutput("Auto/State", getCurrentState().name());
}
```
---

## Where to Go Next

**Linked below are extra resources and disscussions for autonomous performance**

**Competition Strategies**

> **TODO:** The links in this section are not working properly and need to be redone.

---