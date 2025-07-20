# Swerve Trajectory: Getting Started

## Overview

Swerve trajectory combines the power of motion profiling with holonomic movement, letting your robot follow smooth, planned paths while rotating independently. Instead of jerky point-to-point movement, your robot will glide along curves with perfect timing and coordination.

Jump to [Basic Implementation](#3-basic-implementation) for a quick start. However it is highly recommended to read the sections 1-2 for a lower level understanding of trajectories.

**What you'll build:** A **Swerve** robot that smoothly follows pre-planned paths with independent rotation control. This will cover the underlying explanation behind trajectory planning and a simple implementation of trajectory planning and following from wpilib. 

---

## 1. Why Swerve Trajectories?

### The Problem with Basic Movement
```java
// Basic swerve: Get there somehow
new SequentialCommandGroup(
    new InstantCommand(() -> swerve.drive(new ChassisSpeeds(2.0, 0, 0))), // Move forward
    new WaitCommand(1.0), // Wait for 1 second (adjust as needed)
    new InstantCommand(() -> swerve.drive(new ChassisSpeeds(0, 2.0, 0))) // Move sideways
)
```

**Issues:**
- Did not need to rely on vision -> Teams primarily chose full wheel odometry to measure how far they travelled, which could potentially lead to measurment errors.
- Can't avoid obstacles smoothly

However, with tank drives being the primary choice of drive base, this method of striaght-line movements was the common autonomous method in games pre-2022.

### Swerve Trajectory Solution
```java
public Command trajectory() {
    PathPlannerPath path = getStraightLinePath(currentPose, goal, DriveConstants.PATHFINDING_CONSTRAINTS); 
    return new TrajectoryDriveCmd(path, true, true).
}
// Robot follows exact path with precise timing
```

**Benefits:**
- **Predictable paths** - Know exactly where robot will be
- **Smooth motion** - Curved paths instead of sharp corners
- **Independent rotation** - Face any direction while moving
- **Obstacle avoidance** - Plan around field elements

With the introduction of swerve drives and outside libraries such as Pathplanner and Choreo, the bar to acchiving trajectory has been lowered. More and more teams can take advantage of these tools to boost compeittion performance. 

>However, it is crucial to understand what happens at a lower level with trajectory planning. Without understanding some lower level components, it's easy for teams to solve their trajectory issues with the wrong methods due to a lack of understanding. 

**Holonomic Advantage:**

If you recall from [Swerve Control: Control Definitions](./swerve_control.md#control-definitions), Holomonic Controllers, which include swerves operating on trajectory, will not require the robot to face in a specific direction to follow the path, easing time constraints and enabling more game specific actions.
```
Traditional Robot:    Swerve Robot:
     ↑                   ↑ ← Robot can face any direction
     |                   |   while following any path
  [ROBOT]             [ROBOT]
     |                   ↓
     Path = Facing    Path ≠ Facing
```

---

## 2. Understanding Swerve Trajectories

### Key Distinctions

There is a key distinction to make between **Trajectories** and **Paths**
- **Path** - The shape/route (like a road)
    - Used by Pathplanner and Choreo vendors display on gui.  
- **Trajectory** - Path + timing (like a GPS route with arrival times)

**Further Reading and Reference:**  
 [Path Planning and Trajectory Planning Algorithms: a General Overview](https://bia.unibz.it/esploro/outputs/bookChapter/Path-Planning-and-Trajectory-Planning-Algorithms-a-General/991005772345001241). 


### Components of a Swerve Trajectory

Due to the slight differences in how Pathplanner and Choreo define a trajectory, we will be explaining the WPILIB definition of a trajectory.

**Translation Trajectory:**
- X and Y velocity
- X and Y acceleration
- Goal Pose

**Rotation Trajectory:**
- Angular velocity 
- Angular Accel
- Heading

**Combined Result:**
At any time `t`, the trajectory provides a full robot state at that singular moment. This is usually encapsulated within its own object called a **state** or **sample**. This can be then used as target information for our robot to follow in order to stay on the path the user provided.
```java
// At any time t, the trajectory provides:
TrajectoryState state = trajectory.sample(t);
// state.poseMeters     - Where robot should be (x, y, rotation)
// state.velocityMPS    - How fast robot should move
// state.curvatureRPM   - Path curvature at this point
```

### Path Constraints

Path constraints are rules that limit the robot's speed, acceleration, or other properties along specific sections or entire sections of a trajectory. They help give more control over what the drive base should do at a specific area. Similarly to before, there are slight implementation differences across trajectory libraries. WPILIB uses a code based approach shown below while Pathplanner and Chreo prefer to have them defined on their path GUI.

**Common constraint types:**
- **Max Velocity Constraint:** Limits top speed in a region.
- **Max Acceleration Constraint:** Restricts acceleration.
- **Centripetal Acceleration Constraint:** Prevents excessive speed.
- **Region Constraint:** Applies constraints only within a defined area.
- **Custom Constraints:** WPILIB trajectory implementation specifcally allows users to implement their own logics that determines when velocity, acceleration, or region constraints should be applied.

Path constraints can have 1 or more of these constraints combined into one. Here is a WPILIB example:

```java
TrajectoryConfig config = new TrajectoryConfig(2.0, 1.0);

// Slow down in the scoring zone (rectangle from (2,2) to (4,4))
config.addConstraint(new RectangularRegionConstraint(
    new Translation2d(2, 2),           // Bottom-left corner
    new Translation2d(4, 4),           // Top-right corner
    new MaxVelocityConstraint(0.5)     // Limit speed to 0.5 m/s in this region
));

// Generate trajectory with constraints
Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(new Translation2d(3, 3)),
    new Pose2d(5, 5, new Rotation2d(0)),
    config
);
```

### Holomonic Controller

The robot's current state (position and velocity) is used by a holonomic controller object to calculate the desired chassis speeds (vx, vy, omega) needed to follow the trajectory. These speeds are then sent to the robot's drive subsystem, enabling the trajectory.

**Key Steps:**
- The robot's current pose and velocity are passed to the holonomic controller.
- The controller computes the required chassis speeds (vx, vy, omega) to achieve the desired motion.
- These speeds are then sent to the robot's drive subsystem for execution.

```java
// Example: Using a holonomic controller to follow a trajectory
HolonomicDriveController controller = new HolonomicDriveController(
    new PIDController(1.0, 0, 0),    // X controller
    new PIDController(1.0, 0, 0),    // Y controller
    new ProfiledPIDController(2.0, 0, 0, new TrapezoidProfile.Constraints(3.0, 2.0)) // Rotation
);

@Override
public void periodic() {
    if (isFollowingTrajectory) {
        Trajectory.State desiredState = trajectory.sample(timer.get());
        Pose2d currentPose = swerve.getPose();

        ChassisSpeeds speeds = hocontroller.calculate(
            currentPose,
            desiredState,
            desiredState.poseMeters.getRotation()
        );
    }
}
```


**Reference and Further Reading**  
- [WPILib Trajectory Class Reference](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/trajectory/Trajectory.html)
- [WPILib Trajectory Generation Documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/trajectory-generation.html)
- [WPILib Trajectory Constraints Documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/constraints.html)

---

## 3. Basic Implementation

Most teams will choose from vendors such as Pathplanner or Choreo to run their trajectory functions due to their add-on functionality. However it is recommended to first implement WPILIB trajectory. This will enable you to have access to every part of how the trajectory runs and familiarlize yourself with all the parameters available to you to adjust your trajectory.  After running this iteration, it will be easier to integrate in Choreo or Pathlanner as your team chooses.

Below is a simple implementation of WPILIB Trajectory. Follow this guide to get it working. 

### Create Your First Trajectory
1. Create a custom class command that will house your trajectory following code.
2. Make sure to have the following features
    - A trajectory object taken in as a parameter
    - A swerve subsystem object taken in as a parameter
    - Timer for tracking state of a trajectory
3. Constructor that completes any remaining object declarations.

### Define your Holomonic Controller
1. Create a holomonic controller object that is defined in the Command constructor. Use PIDController objects as parameters.
2. Compartively to other PID control systems, start with int type numbers such as 1.0, 2.0, etc. 


### Populate Execute, Initialize, isFinished
1. `Intitalize` should contain all your reset methods for:
    - reseting odometry if it's the first path
    - restarting the internal timer to match with the trajectory time
2. `Execute` should contain:
    - all your trajectory state/swerve measurment readings.
    - The final outputs should be a chassisspeed object that can be fed into the swerve drive.

3. `isFinished` should terminate when:
-  The recorded swerve measurments are in a certain deadband of the goal Pose
- When our timer exceeds the estimated time for our trajectory.


```java
public class TrajectoryCommand extends CommandBase {
    private final SwerveDrive swerve;
    private final Timer timer = new Timer();
    private final Trajectory trajectory;
    private final HolonomicDriveController holonomicController;
    private static final double POSITION_TOLERANCE = 0.10; // meters
    private static final double ROTATION_TOLERANCE = 5.0; // degrees

    public TrajectoryCommand(SwerveDrive swerve, Trajectory trajectory) {
        this.swerve = swerve;
        this.trajectory = trajectory;

        // Holonomic controller for trajectory following
        holonomicController = new HolonomicDriveController(
            new PIDController(1.0, 0, 0),    // X controller
            new PIDController(1.0, 0, 0),    // Y controller
            new ProfiledPIDController(2.0, 0, 0, new TrapezoidProfile.Constraints(3.0, 2.0)) // Rotation
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
        Trajectory.State desiredState = trajectory.sample(timer.get());

        // Get current robot pose
        Pose2d currentPose = swerve.getPose();

        // Use holonomic controller to calculate chassis speeds
        ChassisSpeeds chassisSpeeds = holonomicController.calculate(
            currentPose,
            desiredState,
            desiredState.poseMeters.getRotation()
        );

        swerve.drive(chassisSpeeds);

        // SmartDashboard telemetry for debugging
        SmartDashboard.putNumber("Trajectory Time", timer.get());
        SmartDashboard.putNumber("Desired X", desiredState.poseMeters.getX());
        SmartDashboard.putNumber("Desired Y", desiredState.poseMeters.getY());
        SmartDashboard.putNumber("Desired Rotation", desiredState.poseMeters.getRotation().getDegrees());
        SmartDashboard.putNumber("Robot X", currentPose.getX());
        SmartDashboard.putNumber("Robot Y", currentPose.getY());
        SmartDashboard.putNumber("Robot Rotation", currentPose.getRotation().getDegrees());
        SmartDashboard.putNumber("X Error", currentPose.getX() - desiredState.poseMeters.getX());
        SmartDashboard.putNumber("Y Error", currentPose.getY() - desiredState.poseMeters.getY());
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = swerve.getPose();
        Pose2d goalPose = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters;

        boolean positionClose = currentPose.getTranslation().getDistance(goalPose.getTranslation()) < POSITION_TOLERANCE;
        boolean rotationClose = Math.abs(currentPose.getRotation().getDegrees() - goalPose.getRotation().getDegrees()) < ROTATION_TOLERANCE;

        return timer.get() > trajectory.getTotalTimeSeconds() || (positionClose && rotationClose);
    }
}
```

---

## 5. Creating Autos

Now that we've created our first intial trajectory commands, let's fit it into an autonomous sequence.

### Creating trajectories
1. Create a new trajectory config with a max speed and max acceleration
2. Create a trjaectory with:
    - Start and Goal Poses
    - Translation2d type waypoints (Units in Meters)
```java
//   maxSpeed: The maximum speed for the trajectory (in meters per second).
//   maxAcceleration: The maximum acceleration for the trajectory (in meters per second squared).
TrajectoryConfig config = new TrajectoryConfig(2.0, 1.0); 

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

### (Optional)Constraints for Different Sections
```java
// Slower through tight areas
TrajectoryConfig slowConfig = new TrajectoryConfig(1.0, 0.5);
slowConfig.addConstraint(new RectangularRegionConstraint(
    new Translation2d(1, -1),  // Bottom-left corner
    new Translation2d(3, 1),   // Top-right corner  
    new MaxVelocityConstraint(0.5) // Slow down in this region
));
```

### Apply it to a Drive Trajectory Command
Here is a full example usage as a full Command. This can be used in a chain of trajectory drive commands as shown in Real-World example below.

```java
// Single-command autonomous routine using TrajectoryCommand
public Command testPath1(SwerveDrive swerve) {
    TrajectoryConfig config = new TrajectoryConfig(2.0, 1.0);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(2, 1), new Translation2d(3, -1)),
        new Pose2d(4, 0, new Rotation2d(Math.PI)),
        config
    );

    return new TrajectoryCommand(swerve, trajectory);
}
```

---

## 6. Real-World Examples

### Simple Autonomous Routine
```java
public class SimpleAuto extends SequentialCommandGroup {
    public SimpleAuto(SwerveDrive swerve) {
        // Trajectory config: max speed 2 m/s, max accel 1 m/s^2
        TrajectoryConfig config = new TrajectoryConfig(2.0, 1.0);

        // First trajectory: leave starting position
        Trajectory leaveTarmac = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 0)),
            new Pose2d(2, 0, new Rotation2d(0)),
            config
        );

        // Second trajectory: return and face backward
        Trajectory returnToShoot = TrajectoryGenerator.generateTrajectory(
            new Pose2d(2, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 0)),
            new Pose2d(0, 0, new Rotation2d(Math.PI)),
            config
        );

        addCommands(
            new TrajectoryCommand(swerve, leaveTarmac),
            new TrajectoryCommand(swerve, returnToShoot)
        );
    }
}
```

---

## 7. Tuning and Debugging

### Common Issues & Solutions

### PID Tuning Guidelines
>**Important** Make sure your swerve drive is tuned properly first before tuning your holonomic controller. See [Chapter 4: Simple Profiling](../Chapter_4/simple_profiling.md) for tunning a velocity controller. Tuning the holonomic controller first will lead to issues in following your trajectory properly.

It's recommended to start big incrementing by ints or in 10s decimals places when tunning.

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
    0.0   // D: Avoid
);
```

**Problem:** Robot doesn't follow path accurately  
- **Possible Fix:** Tune PID controllers, check odometry calibration. Are the drive velocity controllers tuned properly?

**Problem:** Robot is not accurate to real world.  
- **Possible Fix:** Double check field measurements, calibrate all sensors, and confirm the coordinate system matches your field setup. Even small measurement errors can accumulate and lead to noticeable trajectory inaccuracies.

**Problem:** Path is too aggressive/jerky  
- **Possible Fix:** Lower max velocity/acceleration in TrajectoryConfig.

**Problem:** Robot overshoots waypoints  
- **Possible Fix:** Increase derivative gains, add velocity constraints. Use SmartDashboard to check if your current pose is lagging behind your goal pose.

**Problem:** Rotation lags behind translation  
- **Possible Fix:** Tune theta controller separately, check angular constraints.

**Problem:** Robot drifts off path over time  
- **Possible Fix:** Improve odometry (better wheel characterization, vision correction).

### **Success Criteria:**
- Robot follows planned path within 10cm accuracy
- Smooth acceleration and deceleration
- Consistent timing for autonomous coordination
- Independent rotation works smoothly


---

## 8. Advanced Techniques

### Dynamic Trajectory Generation
Below is a simple Drive to Pose that can be useful for games where the player needs to autonomously align with scoring or loading positions.

```java
public Command autoTargetPose(SwerveDrive swerve, Pose2d target) {
    // Create new trajectory from current position
    Trajectory dynamicPath = TrajectoryGenerator.generateTrajectory(
        swerve.getPose(),      // Start from where we are now
        List.of(),
        target,                // Go to new target
        config
    );
    
    // Start following immediately
    return new TrajectoryCommand(swerve, dynamicPath)
}
```
---

## Links for further reference

**Visual Path Planning**
- [PathPlanner](https://pathplanner.dev/) - Visual trajectory designer
- [Choreo](https://sleipnirgroup.github.io/Choreo/) - Advanced trajectory optimization