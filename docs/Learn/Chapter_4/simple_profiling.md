# Motion Profiling: Getting Started

## Overview

Motion profiling creates smooth, predictable movement by planning the entire motion before it happens. Instead of jerky start-stop movement, your mechanisms will accelerate smoothly, cruise at optimal speed, then decelerate perfectly to the target.

**What you'll build:** Mechanisms that move with professional smoothness - no more jarring stops or overshooting.

---

## 1. Why Motion Profiling?

### The Problem with Basic PID
```java
// Basic PID: Jerky, unpredictable movement
setPosition(currentPos + 24); // Jump command - mechanism lurches forward
```

**Issues:**
- Sudden acceleration stresses mechanisms
- Inconsistent timing makes coordination difficult  
- Hard to predict when movement will finish
- Can cause oscillation and overshoot

### Motion Profiling Solution
```java
// Motion profiling: Smooth, planned movement
TrapezoidProfile.State goal = new TrapezoidProfile.State(targetPos, 0);
// Automatically generates smooth acceleration → cruise → deceleration
```

**Benefits:**
- **Predictable timing** - Know exactly when movement finishes
- **Smooth acceleration** - Gentle on mechanisms and game pieces
- **Coordinated motion** - Multiple mechanisms move in sync
- **Reduced wear** - Less mechanical stress

---

## 2. Understanding Motion Profiles

### The Trapezoidal Profile
A motion profile defines **position**, **velocity**, and **acceleration** over time:

```
Velocity
   ^
   |     /‾‾‾‾‾‾‾\     ← Cruise phase (constant velocity)
   |    /         \
   |   /           \   ← Acceleration/deceleration phases  
   |  /             \
   |_/_______________\____→ Time
     ↑               ↑
   Start            End
```

**Three Phases:**
1. **Acceleration** - Ramp up to max velocity
2. **Cruise** - Maintain constant velocity  
3. **Deceleration** - Ramp down to stop at target

### Key Parameters
- **Max Velocity** - Top speed during cruise phase
- **Max Acceleration** - How quickly we ramp up/down
- **Start State** - Current position and velocity
- **End State** - Target position and velocity (usually 0)

### Analogy: Driving a Car with and without Motion Profiling

Imagine you're driving a car to a stop sign:

- **Without Motion Profiling (Basic PID Only):**  
    You slam the gas pedal to reach the stop sign as quickly as possible, then stomp on the brakes at the last moment. The car lurches forward, passengers get tossed around, and you might overshoot or stop short. This is how basic PID works—reacting to errors without planning the whole journey.

- **With Motion Profiling + PID:**  
    Instead, you plan your drive: gently accelerate, cruise at a comfortable speed, then smoothly slow down so you stop exactly at the sign. Motion profiling creates this plan—defining how fast to go and when to speed up or slow down. PID then acts like your cruise control, making small adjustments to keep you on the planned path.

**Summary:**  
Motion profiling is like planning your route and speed ahead of time for a smooth, predictable trip. PID is your assistant, making sure you follow that plan precisely, correcting for hills or bumps along the way.

---

## 3. Basic Motion Profile Implementation

### Step 1: Create the Profile
These are the necessary components to setup a motion profile subsystem using available hardware.
```java
public class ElevatorSubsystem extends SubsystemBase {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final PIDController pidController;
    
    // Motion profile constraints
    private final TrapezoidProfile.Constraints constraints = 
        new TrapezoidProfile.Constraints(
            2.0,  // Max velocity (inches/sec)
            4.0   // Max acceleration (inches/sec²)
        );
    
    private TrapezoidProfile profile;
    private Timer timer = new Timer();
    private TrapezoidProfile.State goal;
    
    public ElevatorSubsystem() {
        motor = new CANSparkMax(1, MotorType.kBrushless);
        encoder = motor.getEncoder();
        pidController = new PIDController(0.1, 0.0, 0.01);
    }
}
```

### Step 2: Start a Profiled Movement

Write these methods within your subsystems so existing commands or periodics can set position for your mechanism.
```java
public void setTargetPosition(double targetPosition) {
    // Current state (where we are now)
    TrapezoidProfile.State current = new TrapezoidProfile.State(
        encoder.getPosition(),
        getVelocity()
    );
    
    // Goal state (where we want to be)
    goal = new TrapezoidProfile.State(targetPosition, 0);
    
    // Create the profile
    profile = new TrapezoidProfile(constraints, goal, current);
    
    // Start timing
    timer.restart();
}

private double getVelocity() {
    return encoder.getVelocity() / 60.0; // Convert RPM to inches/sec
}
```

### Step 3: Follow the Profile

In your subsystem periodic, this logic will run your looped motion profile.
```java
@Override
public void periodic() {
    if (profile != null) {
        // Get the current setpoint from the profile
        TrapezoidProfile.State setpoint = profile.calculate(timer.get());
        
        // Use PID to follow the setpoint
        double pidOutput = pidController.calculate(
            encoder.getPosition(), 
            setpoint.position
        );
        
        // Add feedforward for smooth tracking
        double feedforward = setpoint.velocity * 0.1; // kV gain
        
        motor.set(pidOutput + feedforward);
        
        // Check if profile is complete
        if (profile.isFinished(timer.get())) {
            profile = null;
            timer.stop();
        }
    }
}

public boolean isAtTarget() {
    return profile == null || profile.isFinished(timer.get());
}
```

---

## 4. Standard Implementation: WPILib's ProfiledPIDController

### Simplified Implementation
WPILib combines profiling + PID into one easy class:

```java
public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    
    // All-in-one profiled PID controller
    private final ProfiledPIDController controller;
    
    public ArmSubsystem() {
        motor = new CANSparkMax(1, MotorType.kBrushless);
        encoder = motor.getEncoder();
        
        // Create constraints
        TrapezoidProfile.Constraints constraints = 
            new TrapezoidProfile.Constraints(
                Math.PI,     // Max velocity (rad/s)
                2 * Math.PI  // Max acceleration (rad/s²)
            );
        
        // Create profiled PID controller
        controller = new ProfiledPIDController(
            1.0, 0.0, 0.1,    // PID gains
            constraints        // Motion constraints
        );
        
        controller.setTolerance(0.05); // Position tolerance
    }
    
    public void setTargetAngle(double targetRadians) {
        controller.setGoal(targetRadians);
    }
    
    @Override
    public void periodic() {
        // Calculate output (automatically handles profiling + PID)
        double output = controller.calculate(encoder.getPosition());
        motor.set(output);
    }
    
    public boolean atTarget() {
        return controller.atGoal();
    }
}
```


---

## 5. Choosing Profile Parameters

### Max Velocity Guidelines
**Too Fast:**
- Mechanism can't keep up
- Control becomes unstable
- Overshooting at end

**Too Slow:**
- Wastes time
- Looks sluggish

**Good Starting Point:** 75% of theoretical max velocity

```java
// Example: Elevator can physically move 4 inches/sec
double maxVelocity = 3.0; // Use 75% for safety margin
```

### Max Acceleration Guidelines
**Too High:**
- Jerky motion defeats the purpose
- Mechanisms stress and wear
- Game pieces might fall

**Too Low:**
- Takes forever to reach max speed
- Profile becomes mostly acceleration/deceleration

**Good Starting Point:** Time to max velocity = 0.5-1.0 seconds

```java
// Example: Reach 3.0 in/s in 0.75 seconds
double maxAcceleration = 3.0 / 0.75; // = 4.0 in/s²
```

---

## 6. Real-World Examples

### Elevator System
```java
// 36-inch travel elevator
TrapezoidProfile.Constraints elevatorConstraints = 
    new TrapezoidProfile.Constraints(
        24.0,  // Max velocity: 24 in/s (2 feet/sec)
        48.0   // Max acceleration: 48 in/s² (0.5 sec to max speed)
    );

ProfiledPIDController elevatorController = new ProfiledPIDController(
    0.2, 0.0, 0.05,     // PID gains
    elevatorConstraints
);
```

### Arm System  
```java
// 180-degree arm rotation
TrapezoidProfile.Constraints armConstraints = 
    new TrapezoidProfile.Constraints(
        Math.PI,      // Max velocity: π rad/s (180°/sec)  
        2 * Math.PI   // Max acceleration: 2π rad/s² (0.5 sec to max)
    );

ProfiledPIDController armController = new ProfiledPIDController(
    2.0, 0.0, 0.1,    // PID gains (higher for rotational)
    armConstraints
);
```

### Shooter Hood
```java
// Small, precise adjustments
TrapezoidProfile.Constraints hoodConstraints = 
    new TrapezoidProfile.Constraints(
        0.5,  // Max velocity: 0.5 rad/s (slow and precise)
        1.0   // Max acceleration: 1.0 rad/s²
    );
```

---

## 7. Advanced Features

### Custom End Velocity
```java
// Don't stop at target - useful for handoffs
TrapezoidProfile.State goal = new TrapezoidProfile.State(
    targetPosition, 
    1.0  // End with 1.0 velocity instead of stopping
);
```

### Coordinated Multi-Axis Movement
```java
public class IntakeSequence extends CommandBase {
    public IntakeSequence(ArmSubsystem arm, WristSubsystem wrist) {
        // Both movements finish at the same time
        addCommands(
            new MoveArmCommand(arm, Math.PI/4).withTimeout(2.0),
            new MoveWristCommand(wrist, Math.PI/6).withTimeout(2.0)
        );
    }
}
```

### Dynamic Constraint Adjustment
```java
public void setSpeed(boolean fastMode) {
    TrapezoidProfile.Constraints newConstraints;
    
    if (fastMode) {
        newConstraints = new TrapezoidProfile.Constraints(4.0, 8.0);
    } else {
        newConstraints = new TrapezoidProfile.Constraints(1.0, 2.0);
    }
    
    controller.setConstraints(newConstraints);
}
```

---

## 8. Debugging Motion Profiles

### Essential Telemetry
```java
@Override
public void periodic() {
    // Current state
    SmartDashboard.putNumber("Position", encoder.getPosition());
    SmartDashboard.putNumber("Velocity", getVelocity());
    
    // Profile state
    TrapezoidProfile.State setpoint = controller.getSetpoint();
    SmartDashboard.putNumber("Profile Position", setpoint.position);
    SmartDashboard.putNumber("Profile Velocity", setpoint.velocity);
    
    // Control output
    double output = controller.calculate(encoder.getPosition());
    SmartDashboard.putNumber("Motor Output", output);
    SmartDashboard.putBoolean("At Goal", controller.atGoal());
    
    motor.set(output);
}
```

### Common Issues & Solutions

**Problem:** Motion is still jerky
- **Solution:** Lower max acceleration, check PID tuning

**Problem:** Never reaches target precisely  
- **Solution:** Add feedforward term, tune PID gains

**Problem:** Takes too long to complete
- **Solution:** Increase max velocity/acceleration within safe limits

**Problem:** Overshoots target
- **Solution:** Lower max velocity, increase D gain, add velocity feedforward

**Problem:** Oscillates around target
- **Solution:** Decrease P gain, increase D gain

---

## 9. Practice Project

**Build this step-by-step:**

1. **Basic Profile** - Create simple position-to-position movement
2. **Tune Parameters** - Adjust velocity/acceleration for smooth motion  
3. **Add Feedforward** - Improve tracking accuracy
4. **Multiple Positions** - Create commands for common positions
5. **Coordinated Motion** - Combine multiple mechanisms
6. **Dynamic Constraints** - Adjust speed based on game state

**Success Criteria:**
- Smooth acceleration and deceleration
- Consistent timing for same distance moves
- Accurate positioning at target
- No mechanical stress or game piece dropping

---

## Where to Go Next

**More resources:**

**🎯 Advanced Profiling**
- [WPILib Trajectory Generation](https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-generation.html) - Multi-dimensional paths
- [Custom Profile Shapes](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html) - S-curves and other profiles

**🔧 Feedforward Control**  
- [Feedforward Characterization](https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/index.html) - Physics-based control
- [Elevator Feedforward](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html#elevatorFeedforward) - Gravity compensation

**📊 Multi-DOF Systems**
- [Differential Drive Trajectories](https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/index.html) - Robot path following
- [Swerve Trajectories](https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/trajectory-tutorial-overview.html) - Holonomic motion

**⚡ Motor Controller Integration**
- [Phoenix Motion Magic](https://v6.docs.ctr-electronics.com/) - Hardware-based profiling
- [REV Smart Motion](https://docs.revrobotics.com/) - SparkMax motion control

---


**Next steps:** Advanced trajectory optimization and feedforward control using [WPILib's trajectory documentation](https://docs.wpilib.org/en/stable/docs/software/pathplanning/index.html).