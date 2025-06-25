# Swerve Trajectory: Getting Started

## Overview

Swerve trajectory combines the power of motion profiling with holonomic movement, letting your robot follow smooth, planned paths while rotating independently. Instead of jerky point-to-point movement, your robot will glide along curves with perfect timing and coordination.

**What you'll build:** A robot that smoothly follows pre-planned paths with independent rotation control.

**Next steps:** Advanced path planning and real-time trajectory generation using [WPILib's PathPlanner](https://pathplanner.dev/) and [trajectory documentation](https://docs.wpilib.org/en/stable/docs/software/pathplanning/index.html).

---

## 1. Why Swerve Trajectories?

### The Problem with Basic Movement
```java
// Basic swerve: Get there somehow
drive(new ChassisSpeeds(2.0, 0, 0)); // Move forward
// Wait some time...
drive(new ChassisSpeeds(0, 2.0, 0)); // Move sideways
// Robot path is unpredictable, timing is inconsistent
```

**Issues:**
- Unpredictable paths waste time and space
- Hard to coordinate with other mechanisms
- Can't avoid obstacles smoothly
- Inconsistent autonomous performance

### Swerve Trajectory Solution
```java
// Trajectory: Smooth, planned path with perfect timing
Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    startPose, List.of(waypoint1, waypoint2), endPose, config
);
// Robot follows exact path with precise timing
```

**Benefits:**
- **Predictable paths** - Know exactly where robot will be
- **Smooth motion** - Curved paths instead of sharp corners
- **Perfect timing** - Coordinate with shooters, intakes, etc.
- **Independent rotation** - Face any direction while moving
- **Obstacle avoidance** - Plan around field elements

---

## 2. Understanding Swerve Trajectories

### Key Concepts

**Trajectory vs Path:**
- **Path** - The shape/route (like a road)
- **Trajectory** - Path + timing (like a GPS route with arrival times)

**Holonomic Advantage:**
```
Traditional Robot:    Swerve Robot:
     ↑                   ↑ ← Robot can face any direction
     |                   |   while following any path
  [ROBOT]             [ROBOT]
     |                   ↓
     Path = Facing    Path ≠ Facing
```

### Components of a Swerve Trajectory

**Translation Trajectory:**
- X position over time
- Y position over time  
- X velocity over time
- Y velocity over time

**Rotation Trajectory:**
- Robot heading over time
- Angular velocity over time

**Combined Result:**
```java
// At any time t, the trajectory provides:
TrajectoryState state = trajectory.sample(t);
// state.poseMeters     - Where robot should be (x, y, rotation)
// state.velocityMPS    - How fast robot should move
// state.curvatureRPM   - Path curvature at this point
```

---

## 3. Basic Implementation

### Step 1: Create Your First Trajectory
```java
public class AutonomousCommand extends CommandBase {
    private final SwerveDrive swerve;
    private final Timer timer = new Timer();
    private final Trajectory trajectory;
    
    public AutonomousCommand(SwerveDrive swerve) {
        this.swerve = swerve;
        
        // Create trajectory configuration
        TrajectoryConfig config = new TrajectoryConfig(
            2.0,  // Max velocity (m/s)
            1.0   // Max acceleration (m/s²)
        );
        
        // Define start and end poses
        Pose2d start = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d end = new Pose2d(3, 2, new Rotation2d(Math.PI/2));
        
        // Generate trajectory
        trajectory = TrajectoryGenerator.generateTrajectory(
            start,
            List.of(), // No intermediate waypoints for now
            end,
            config
        );
        
        addRequirements(swerve);
    }
    
    @Override
    public void initialize() {
        timer.restart();
        swerve.resetOdometry(trajectory.getInitialPose());
    }
    
    @Override
    public void execute() {
        // Get current trajectory state
        TrajectoryState desiredState = trajectory.sample(timer.get());
        
        // Convert to chassis speeds (no rotation control yet)
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            desiredState.velocityMetersPerSecond * desiredState.poseMeters.getRotation().getCos(),
            desiredState.velocityMetersPerSecond * desiredState.poseMeters.getRotation().getSin(),
            0 // No rotation for now
        );
        
        swerve.drive(chassisSpeeds);
    }
    
    @Override
    public boolean isFinished() {
        return timer.get() > trajectory.getTotalTimeSeconds();
    }
}
```

### Step 2: Add Closed-Loop Control
The above is open-loop (no feedback). Add PID control for accuracy:

```java
public class FollowTrajectoryCommand extends CommandBase {
    private final SwerveDrive swerve;
    private final Timer timer = new Timer();
    private final Trajectory trajectory;
    
    // PID controllers for trajectory following
    private final PIDController xController = new PIDController(1.0, 0, 0);
    private final PIDController yController = new PIDController(1.0, 0, 0);
    private final PIDController thetaController = new PIDController(2.0, 0, 0);
    
    public FollowTrajectoryCommand(SwerveDrive swerve, Trajectory trajectory) {
        this.swerve = swerve;
        this.trajectory = trajectory;
        
        // Theta controller wraps around at ±π
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(swerve);
    }
    
    @Override
    public void initialize() {
        timer.restart();
        swerve.resetOdometry(trajectory.getInitialPose());
    }
    
    @Override
    public void execute() {
        // Get where we should be
        TrajectoryState desiredState = trajectory.sample(timer.get());
        
        // Get where we actually are
        Pose2d currentPose = swerve.getPose();
        
        // Calculate feedforward velocities
        double vx = desiredState.velocityMetersPerSecond * 
                   desiredState.poseMeters.getRotation().getCos();
        double vy = desiredState.velocityMetersPerSecond * 
                   desiredState.poseMeters.getRotation().getSin();
        
        // Add PID corrections
        vx += xController.calculate(currentPose.getX(), desiredState.poseMeters.getX());
        vy += yController.calculate(currentPose.getY(), desiredState.poseMeters.getY());
        
        double omega = thetaController.calculate(
            currentPose.getRotation().getRadians(),
            desiredState.poseMeters.getRotation().getRadians()
        );
        
        // Drive the robot
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, omega);
        swerve.drive(chassisSpeeds);
    }
    
    @Override
    public boolean isFinished() {
        return timer.get() > trajectory.getTotalTimeSeconds();
    }
}
```

---

## 4. WPILib's SwerveControllerCommand

### Simplified Implementation
WPILib provides a ready-made command for trajectory following:

```java
public class AutoRoutine extends SequentialCommandGroup {
    public AutoRoutine(SwerveDrive swerve) {
        // Create trajectory
        TrajectoryConfig config = new TrajectoryConfig(2.0, 1.0);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 1)),
            new Pose2d(3, 0, new Rotation2d(0)),
            config
        );
        
        // Create controller command
        SwerveControllerCommand swerveCommand = new SwerveControllerCommand(
            trajectory,
            swerve::getPose,           // Robot pose supplier
            swerve.getKinematics(),    // Swerve kinematics
            
            // PID controllers for x, y, and rotation
            new PIDController(1.0, 0, 0),    // X controller
            new PIDController(1.0, 0, 0),    // Y controller  
            new PIDController(2.0, 0, 0),    // Rotation controller
            
            swerve::setModuleStates,   // Module states consumer
            swerve                     // Subsystem requirement
        );
        
        addCommands(
            new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
            swerveCommand,
            new InstantCommand(() -> swerve.drive(new ChassisSpeeds())) // Stop
        );
    }
}
```

---

## 5. Creating Complex Paths

### Adding Waypoints
```java
// Curved path through multiple points
Trajectory complexPath = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),           // Start
    List.of(
        new Translation2d(1, 1),                    // Waypoint 1
        new Translation2d(2, -1),                   // Waypoint 2
        new Translation2d(3, 0)                     // Waypoint 3
    ),
    new Pose2d(4, 1, new Rotation2d(Math.PI)),     // End facing backward
    config
);
```

### Constraints for Different Sections
```java
// Slower through tight areas
TrajectoryConfig slowConfig = new TrajectoryConfig(1.0, 0.5);
slowConfig.addConstraint(new RectangularRegionConstraint(
    new Translation2d(1, -1),  // Bottom-left corner
    new Translation2d(3, 1),   // Top-right corner  
    new MaxVelocityConstraint(0.5) // Slow down in this region
));
```

### Independent Rotation Control
```java
// Rotate while moving - holonomic advantage!
public class SpinWhileMoving extends SequentialCommandGroup {
    public SpinWhileMoving(SwerveDrive swerve) {
        Trajectory path = /* create your path */;
        
        // Follow path AND rotate independently
        addCommands(
            deadline(
                new FollowTrajectoryCommand(swerve, path),      // Primary: follow path
                new RunCommand(() -> {                          // Secondary: spin
                    double omega = Math.sin(Timer.getFPGATimestamp()) * 2.0;
                    swerve.drive(new ChassisSpeeds(0, 0, omega));
                }, swerve)
            )
        );
    }
}
```

---

## 6. Real-World Examples

### Simple Autonomous Routine
```java
public class SimpleAuto extends SequentialCommandGroup {
    public SimpleAuto(SwerveDrive swerve, ShooterSubsystem shooter) {
        // Configuration for normal movement
        TrajectoryConfig config = new TrajectoryConfig(3.0, 2.0);
        
        // Leave starting position
        Trajectory leaveTarmac = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(2, 0, new Rotation2d(0)),
            config
        );
        
        // Return to shoot
        Trajectory returnToShoot = TrajectoryGenerator.generateTrajectory(
            new Pose2d(2, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(0.5, 0, new Rotation2d(Math.PI)), // Face hub
            config
        );
        
        addCommands(
            // Shoot preload
            new ShootCommand(shooter).withTimeout(2.0),
            
            // Drive to pick up ball
            new SwerveControllerCommand(/* leaveTarmac parameters */),
            
            // Return and shoot
            new SwerveControllerCommand(/* returnToShoot parameters */),
            new ShootCommand(shooter).withTimeout(2.0)
        );
    }
}
```

### Defense Evasion
```java
// Avoid defender in center field
TrajectoryConfig avoidanceConfig = new TrajectoryConfig(2.0, 1.5);

// Create constraint to avoid center region
avoidanceConfig.addConstraint(new RectangularRegionConstraint(
    new Translation2d(6, 2),   // Defender zone
    new Translation2d(10, 6),
    new MaxVelocityConstraint(0.0) // Can't enter this area
));

Trajectory avoidDefense = TrajectoryGenerator.generateTrajectory(
    new Pose2d(2, 4, new Rotation2d(0)),
    List.of(
        new Translation2d(6, 1),   // Go around bottom
        new Translation2d(10, 1),
        new Translation2d(14, 4)   // Reach target
    ),
    new Pose2d(16, 4, new Rotation2d(0)),
    avoidanceConfig
);
```

---

## 7. Tuning and Debugging

### Essential Telemetry
```java
@Override
public void periodic() {
    Pose2d currentPose = swerve.getPose();
    
    // Current robot state
    SmartDashboard.putNumber("Robot X", currentPose.getX());
    SmartDashboard.putNumber("Robot Y", currentPose.getY());
    SmartDashboard.putNumber("Robot Rotation", currentPose.getRotation().getDegrees());
    
    // If following trajectory
    if (isFollowingTrajectory) {
        TrajectoryState desired = trajectory.sample(timer.get());
        SmartDashboard.putNumber("Desired X", desired.poseMeters.getX());
        SmartDashboard.putNumber("Desired Y", desired.poseMeters.getY());
        SmartDashboard.putNumber("Trajectory Time", timer.get());
        
        // Track errors
        double xError = currentPose.getX() - desired.poseMeters.getX();
        double yError = currentPose.getY() - desired.poseMeters.getY();
        SmartDashboard.putNumber("X Error", xError);
        SmartDashboard.putNumber("Y Error", yError);
    }
}
```

### Common Issues & Solutions

**Problem:** Robot doesn't follow path accurately
- **Solution:** Tune PID controllers, check odometry calibration

**Problem:** Path is too aggressive/jerky  
- **Solution:** Lower max velocity/acceleration in TrajectoryConfig

**Problem:** Robot overshoots waypoints
- **Solution:** Increase derivative gains, add velocity constraints

**Problem:** Rotation lags behind translation
- **Solution:** Tune theta controller separately, check angular constraints

**Problem:** Robot drifts off path over time
- **Solution:** Improve odometry (better wheel characterization, vision correction)

### PID Tuning Guidelines
```java
// Start with these values, adjust based on robot performance
PIDController xController = new PIDController(
    1.0,  // P: Start here, increase if robot is slow to correct
    0.0,  // I: Usually not needed for trajectory following  
    0.1   // D: Add if oscillating around path
);

// Rotation typically needs higher gains
PIDController thetaController = new PIDController(
    2.0,  // P: Higher for rotation
    0.0,  // I: Avoid unless persistent error
    0.2   // D: Important for smooth rotation
);
```

---

## 8. Advanced Techniques

### Vision-Corrected Trajectories
```java
public class VisionCorrectedTrajectory extends CommandBase {
    @Override
    public void execute() {
        // Get vision correction
        Optional<Pose2d> visionPose = photonVision.getEstimatedPose();
        
        if (visionPose.isPresent()) {
            // Correct odometry with vision
            swerve.addVisionMeasurement(visionPose.get(), Timer.getFPGATimestamp());
        }
        
        // Continue following trajectory with corrected pose
        followTrajectory();
    }
}
```

### Dynamic Trajectory Generation
```java
public void generatePathToTarget(Pose2d target) {
    // Create new trajectory from current position
    Trajectory dynamicPath = TrajectoryGenerator.generateTrajectory(
        swerve.getPose(),      // Start from where we are now
        List.of(),
        target,                // Go to new target
        config
    );
    
    // Start following immediately
    new FollowTrajectoryCommand(swerve, dynamicPath).schedule();
}
```

### State-Dependent Rotation
```java
// Face target while moving
public class FaceTargetWhileMoving extends CommandBase {
    @Override
    public void execute() {
        // Follow translation trajectory
        TrajectoryState state = trajectory.sample(timer.get());
        
        // Calculate rotation to face target
        Translation2d robotToTarget = target.minus(swerve.getPose().getTranslation());
        Rotation2d targetHeading = new Rotation2d(robotToTarget.getX(), robotToTarget.getY());
        
        // Combine trajectory velocity with target-facing rotation
        ChassisSpeeds speeds = new ChassisSpeeds(
            state.velocityMetersPerSecond * state.poseMeters.getRotation().getCos(),
            state.velocityMetersPerSecond * state.poseMeters.getRotation().getSin(),
            thetaController.calculate(swerve.getPose().getRotation().getRadians(), 
                                    targetHeading.getRadians())
        );
        
        swerve.drive(speeds);
    }
}
```

---

## 9. Practice Project

**Build this step-by-step:**

1. **Basic Straight Line** - Simple point A to point B trajectory
2. **Add Waypoints** - Create curved paths through multiple points
3. **Closed-Loop Control** - Add PID feedback for accuracy
4. **Complex Autonomous** - Multi-step routine with trajectories
5. **Independent Rotation** - Face different directions while moving
6. **Dynamic Planning** - Generate trajectories on-the-fly

**Success Criteria:**
- Robot follows planned path within 10cm accuracy
- Smooth acceleration and deceleration
- Consistent timing for autonomous coordination
- Independent rotation works smoothly

---

## Where to Go Next

**Ready for advanced path planning? Explore these:**

**🎯 Visual Path Planning**
- [PathPlanner](https://pathplanner.dev/) - Visual trajectory designer
- [Choreo](https://sleipnirgroup.github.io/Choreo/) - Advanced trajectory optimization

**🔧 Real-Time Planning**
- [PPLIB](https://pathplanner.dev/pplib-getting-started.html) - Real-time path generation
- [Dynamic Constraints](https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-generation.html#constraints) - Adaptive speed limits

**📊 Advanced Control**
- [Ramsete Controller](https://docs.wpilib.org/en/stable/docs/software/pathplanning/ramsete.html) - Nonlinear trajectory following
- [LTV Unicycle Controller](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html) - Optimal trajectory tracking

**⚡ Competition Integration**
- [Auto Selector](https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/choosing-an-autonomous-program.html) - Multiple autonomous routines
- [PathPlanner Autos](https://pathplanner.dev/gui-auto-editor.html) - Complex autonomous sequences

---

**🚀 Ready to plan perfect paths?** Start with simple trajectories, master the basics, then explore visual path planning tools to create competition-winning autonomous routines!