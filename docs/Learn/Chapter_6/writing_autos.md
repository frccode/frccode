I understand now! You want me to reformat the autonomous programming guide to match the style and structure of the Basic Control Theory document. Here's the autonomous guide modeled after your provided format:

# FRC Autonomous Programming

## Overview

This guide teaches you how to design and implement high-performance autonomous routines for FRC robots, covering strategic planning, technical implementation, and systematic testing approaches.

**What you'll build:** A complete autonomous system that can reliably score points, position your robot advantageously, and adapt to different match scenarios.

---

## 1. Why Autonomous Matters?

For competitive FRC teams, autonomous mode is often the difference between winning and losing matches.

**Simple Mobility?** This gets your robot moving but doesn't maximize scoring potential.

```java
// Basic mobility - just drive forward
public Command getMobilityAuto() {
    return new DriveForward(driveSubsystem, 3.0); // Drive 3 meters
}
```
This approach works but leaves significant points on the table and provides no strategic advantage.

**Static Autonomous?** This scores a preloaded game piece but doesn't adapt to field conditions.

```java
// Static preload scoring
public Command getPreloadAuto() {
    return new SequentialCommandGroup(
        new ScorePreload(superstructure),
        new DriveForward(driveSubsystem, 3.0)
    );
}
```

However, static autonomous routines can't adapt to alliance partners, field conditions, or strategic needs that change throughout a competition.

**Strategic Autonomous** Autonomous routines designed around game analysis, alliance coordination, and robust execution provide:

- **Maximum point potential:** Score multiple game pieces efficiently
- **Strategic positioning:** End in advantageous locations for teleop
- **Alliance coordination:** Complement rather than conflict with partners  
- **Tiebreaker advantage:** Auto points often determine close matches
- **Consistent execution:** Work reliably across different field conditions

**Real Benefits**
- Significant competitive advantage in qualification matches
- Higher alliance selection value 
- Better playoff performance through coordination
- Psychological advantage over opponents

---

## 2. Strategic Foundation

### Game Analysis - "What should we accomplish?"
Before writing any code, analyze what autonomous can achieve:

**Key Questions:**
- What bonus points are available only during autonomous?
- How do autonomous points compare to teleop scoring rates?
- What field positioning advantages exist?
- How does autonomous performance affect tiebreakers?

**Example Analysis (2023 Charged Up):**
```
Mobility: 3 points (auto only)
Game pieces: 3-6 points depending on level  
Balance: 8-12 points depending on robot count
Tiebreaker: Auto points are 4th criteria (critical!)

Theoretical maximum: ~27 points in 15 seconds
Realistic target: 15-20 points consistently
```

### Time and Motion Study
```java
// Example timing breakdown for 2023
// Preload score: 2.0 seconds
// Drive to game piece: 3.5 seconds  
// Intake game piece: 1.5 seconds
// Return and score: 4.0 seconds
// Drive to balance: 3.0 seconds
// Balance: 1.0 seconds
// Total: 15.0 seconds (fully utilized)
```

### Alliance Considerations - "Who are we playing with?"

**Qualification Matches:**
- Most alliance partners provide minimal coordination
- Design routines that maximize your individual contribution
- Avoid areas where conflicts are likely

**Playoff Matches:**  
- Playing with skilled, coordinated alliance partners
- Design complementary routines (who does what?)
- Enable multiple strategic options

---

## 3. Architecture and Hardware Setup

### Robot Design for Autonomous Success
Certain design choices significantly impact autonomous capability:

**Enabling Design Features:**
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
- Clear sight lines to AprilTags
- Multiple cameras for redundancy  
- Protected from game piece interference

**Fast, Consistent Mechanisms:**
- Minimize setup/transition times
- Repeatable positioning
- Robust sensor feedback

### Foundation Layer Implementation
```java
// Motor controller setup with voltage compensation
public class DriveSubsystem extends SubsystemBase {
    private final TalonFX leftMotor = new TalonFX(1);
    private final TalonFX rightMotor = new TalonFX(2);
    
    public DriveSubsystem() {
        // Essential for consistent autonomous performance
        leftMotor.configVoltageCompSaturation(12.0);
        leftMotor.enableVoltageCompensation(true);
        
        rightMotor.configVoltageCompSaturation(12.0);  
        rightMotor.enableVoltageCompensation(true);
    }
}
```

---

## 4. Control System Implementation

### Layer 1: Motor Control
The foundation of autonomous control starts with precise motor control:

**TalonFX Voltage Compensation:**
```java
// Consistent performance regardless of battery voltage
talonFX.configVoltageCompSaturation(12.0);
talonFX.enableVoltageCompensation(true);
```

**Motion Profiling Setup:**
```java
// Smooth, predictable motion
TalonFXConfiguration config = new TalonFXConfiguration();
config.slot0.kP = 0.1;
config.slot0.kI = 0.001;  
config.slot0.kD = 0.01;
config.slot0.kF = 0.05;  // Feedforward for velocity

// Motion Magic for smooth profiles
config.motionCruiseVelocity = 15000; // sensor units per 100ms
config.motionAcceleration = 6000;    // sensor units per 100ms per second
```

### Layer 2: Localization and Sensors

**Wheel Odometry Calibration:**
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

## 5. Systematic Tuning Process

### Step 1: Motor Control Tuning
Using visual feedback for optimal performance:

**Feedforward Tuning (kS, kV, kA):**
```java
// 1. Find static friction (kS)
// Increase voltage until mechanism just starts moving
double kS = findMinimumVoltageToMove();

// 2. Find velocity feedforward (kV)  
// Run at constant velocity, measure voltage needed
double kV = testVoltage / testVelocity;

// 3. Tune visually - setpoint vs actual velocity should be parallel
```

**PID Tuning:**
```java
// 4. Add proportional gain until oscillation
// 5. Back off by 25-50%  
// 6. Add derivative to reduce overshoot
// 7. Add integral only if steady-state error exists
```

### Step 2: Localization Tuning

**Camera Calibration:**
```java
// Lens calibration first - use checkerboard patterns
public void calibrateLens() {
    // Follow PhotonVision calibration process
    // Bad lens calibration makes position calibration impossible
}

// Position calibration second
public void calibratePosition() {
    // Start with measured physical offsets
    // Test close and far from AprilTags
    // Verify detected position matches actual position
}
```

**Sensor Fusion Tuning:**
```java
// Balance accuracy vs stability
public void tuneSensorFusion() {
    // Increase vision trust until jitter becomes unacceptable
    // Decrease until path following performance degrades
    // Find optimal balance point
}
```

### Step 3: Path Following Tuning
```java
// Start with aggressive P terms
xController.setP(2.0);
yController.setP(2.0);
rotationController.setP(1.0);

// Increase until jitter occurs during path following
// Back off to stable values
// Tune BEFORE adding vision - vision can introduce oscillations
```

---

## 6. State Machine Implementation

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

## 7. Autonomous Routine Scripting

### Command-Based Autonomous Structure
```java
public class ThreePieceAuto extends SequentialCommandGroup {
    public ThreePieceAuto(
        DriveSubsystem drive, 
        SuperstructureSubsystem superstructure,
        IntakeSubsystem intake
    ) {
        addCommands(
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
}
```

### Path Planning Integration
```java
// Using PathPlanner for trajectory generation
public static final HashMap<String, Command> eventMap = new HashMap<>();

static {
    eventMap.put("startIntake", new StartIntake(intake));
    eventMap.put("stopIntake", new StopIntake(intake));
    eventMap.put("prepareScore", new PrepareScore(superstructure));
}

public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> resetOdometry()),
        AutoBuilder.followPath(
            PathPlannerPath.fromPathFile("ThreePiecePath")
        )
    );
}
```

---

## 8. Testing and Validation

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

## 9. Essential Debugging Tips

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

### Common Problems & Solutions

**Problem:** Robot doesn't follow paths accurately
- **Solution:** Check localization first - bad pose = bad path following
- **Debug:** Plot commanded vs actual path, check for sensor drift

**Problem:** Autonomous works in practice but fails in competition  
- **Solution:** Test with different lighting, carpet conditions, battery levels
- **Debug:** Log environmental factors, compare practice vs competition data

**Problem:** Timing is inconsistent between runs
- **Solution:** Check for blocking commands, verify mechanism speeds
- **Debug:** Add timing telemetry to each command

**Problem:** Robot collides with alliance partners
- **Solution:** Review path planning, add more conservative routing
- **Debug:** Analyze field positioning data from matches

---

## 10. Advanced Techniques

### Dynamic Path Planning
```java
// Adapt paths based on field conditions
public Command getAdaptiveAuto() {
    return new ConditionalCommand(
        getAggressiveThreePiece(),
        getConservativeTwoPiece(),
        () -> alliancePartnersHaveReliableAuto()
    );
}
```

### Vision-Assisted Scoring
```java  
public class VisionAssistedScore extends CommandBase {
    @Override
    public void execute() {
        Optional<PhotonTrackedTarget> target = getTarget();
        
        if (target.isPresent()) {
            // Use vision for final alignment
            Transform3d robotToTarget = target.get().getBestCameraToTarget();
            ChassisSpeeds correction = calculateCorrectionSpeeds(robotToTarget);
            driveSubsystem.drive(correction);
        }
    }
}
```

### Failure Recovery
```java
// Handle missed game pieces gracefully
public class RobustIntakeCommand extends CommandBase {
    private final Timer timeoutTimer = new Timer();
    
    @Override
    public void initialize() {
        timeoutTimer.restart();
    }
    
    @Override
    public boolean isFinished() {
        return gamePieceAcquired() || timeoutTimer.hasElapsed(2.0);
    }
    
    @Override
    public void end(boolean interrupted) {
        if (!gamePieceAcquired()) {
            // Switch to backup autonomous routine
            CommandScheduler.getInstance().schedule(getBackupAuto());
        }
    }
}
```

---

## 11. Practice Project

**Build this step-by-step:**

### Phase 1: Foundation (Week 1-2)
1. **Swerve Drive Setup** - Get basic driving working with odometry
2. **Path Following** - Implement and tune PID controllers for trajectory following  
3. **Vision Pipeline** - Set up AprilTag detection and pose estimation

### Phase 2: Mechanisms (Week 3-4)
1. **State Machine** - Implement superstructure state management
2. **Mechanism Control** - Tune all mechanism PID controllers
3. **Sensor Integration** - Add game piece detection, limit switches

### Phase 3: Integration (Week 5-6)  
1. **Simple Autonomous** - One game piece + mobility
2. **Complex Routines** - Multi-piece autonomous with path planning
3. **Alliance Coordination** - Multiple autonomous options

### Phase 4: Refinement (Week 7-8)
1. **Competition Testing** - Test on official field elements
2. **Failure Handling** - Add robust error recovery
3. **Performance Optimization** - Fine-tune for consistency

**Success criteria:**
- Consistently scores planned number of game pieces
- Reliable execution across different field conditions  
- Smooth, professional-looking movement
- Provides strategic advantage in matches

---

## Where to Go Next

**Ready for advanced autonomous? Explore these:**

**🎯 Advanced Path Planning**
- [PathPlanner](https://pathplanner.dev/) - GUI-based path planning with event markers
- [Choreo](https://sleipnirgroup.github.io/Choreo/) - Trajectory optimization and generation

**🤖 System Identification**  
- [SysId Tool](https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/index.html) - Automatically determine optimal control gains
- [Feedforward Characterization](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html) - Physics-based control

**📊 Advanced Control**
- [State-Space Control](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html) - Modern control theory
- [Pose Estimation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/pose.html) - Multi-sensor localization

**⚡ Competition Strategies**
- [Auto Selection](https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/choosing-an-autonomous-program.html) - Dynamic autonomous choosing
- [Alliance Coordination](https://www.chiefdelphi.com/c/technical/programming/12) - Multi-robot autonomous planning

---

**🚀 Ready to dominate autonomous?** Work through the practice project systematically, then apply these concepts to create autonomous routines that give your team a significant competitive advantage!

**Remember:** "There's more to a good auto than fancy code." Focus on systematic implementation, thorough testing, and strategic thinking about what will actually help you win matches.

**Next steps:** Advanced autonomous coordination and competition strategy development.