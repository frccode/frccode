# PID Control: Getting Started

## Overview

PID control is the foundation of precise robot movement. Instead of manually setting motor speeds, PID automatically adjusts power based on where you are versus where you want to be. This guide teaches you the basics to get your mechanisms moving accurately.

**What you'll build:** A position-controlled mechanism (arm, elevator, etc.) that smoothly moves to target positions.

---

## 1. Why Use PID Control?

**The Problem with Manual Control**
```java
// This doesn't work well:
if (currentPosition < targetPosition) {
    motor.set(0.5);  // Always same speed, regardless of distance
}
```

**PID's Smart Solution**
- **Far from target:** Move fast
- **Close to target:** Move slow  
- **At target:** Stop precisely
- **Overshot:** Correct automatically

**Real Benefits**
- Consistent, repeatable positioning
- Automatically adapts to load changes
- Smooth, professional-looking movement

---

## 2. The Three Components Explained

### kP (Proportional) - "How far off am I?"
```java
error = target - current;
output = kP * error;
```

**What it does:** Provides power proportional to distance from target
- **Large error:** High power
- **Small error:** Low power  
- **Zero error:** No power

**Tuning tip:** Start here! Increase until it moves quickly but doesn't overshoot wildly.

### kD (Derivative) - "How fast am I approaching?"
```java
rateOfChange = (currentError - previousError) / deltaTime;
output = kD * rateOfChange;
```

**What it does:** Slows down as you approach the target
- **Approaching fast:** Reduces power (acts like brakes)
- **Approaching slow:** Minimal effect
- **Moving away:** Adds power

**Tuning tip:** Add this if kP alone causes overshoot. Bigger values = gentler approach.

### kI (Integral) - "How long have I been off?"
```java
errorSum += error * deltaTime;
output = kI * errorSum;
```

**What it does:** Eliminates steady-state error over time
- **Persistent small error:** Gradually increases power
- **At target:** Resets to zero

**Warning:** Use sparingly! Can cause instability if too high.

---

## 3. Quick Start Implementation

### Basic Position Control Setup
```java
public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax motor;
    private final PIDController pidController;
    private final RelativeEncoder encoder;
    
    public ArmSubsystem() {
        motor = new CANSparkMax(1, MotorType.kBrushless);
        encoder = motor.getEncoder();
        
        // Start with these values, tune from here
        pidController = new PIDController(0.1, 0.0, 0.01);
        pidController.setTolerance(0.05); // Within 0.05 units = "at target"
    }
    
    public void setTargetPosition(double target) {
        double output = pidController.calculate(encoder.getPosition(), target);
        motor.set(output);
    }
    
    public boolean atTarget() {
        return pidController.atSetpoint();
    }
}
```

### Command-Based Usage
```java
public class MoveArmCommand extends CommandBase {
    private final ArmSubsystem arm;
    private final double targetPosition;
    
    public MoveArmCommand(ArmSubsystem arm, double target) {
        this.arm = arm;
        this.targetPosition = target;
        addRequirements(arm);
    }
    
    @Override
    public void execute() {
        arm.setTargetPosition(targetPosition);
    }
    
    @Override
    public boolean isFinished() {
        return arm.atTarget();
    }
}
```

---

## 4. Step-by-Step Tuning Process

### Step 1: Start with P Only
```java
pidController = new PIDController(0.0, 0.0, 0.0);
```

1. Set kI and kD to 0
2. Gradually increase kP until the system moves toward the target
3. Keep increasing until it oscillates around the target
4. Reduce kP by 25-50%

**Good kP signs:**
- Moves quickly to target
- Small overshoot or oscillation
- Settles near (but not exactly at) target

### Step 2: Add D for Smoothness
```java
pidController = new PIDController(yourKp, 0.0, 0.01);
```

1. Start with small kD (try 10-100x smaller than kP)
2. Increase until overshoot is eliminated
3. Too much kD makes response sluggish

**Good kD signs:**
- Smooth approach to target
- Little to no overshoot
- Still reaches target quickly

### Step 3: Add I for Precision (Optional)
```java
pidController = new PIDController(yourKp, 0.001, yourKd);
```

1. Only add if system doesn't reach exact target
2. Start very small (1000x smaller than kP)
3. Increase slowly until steady-state error disappears

**Warning signs (kI too high):**
- Oscillation that gets worse over time
- Overshoot that takes long to settle

---

## 5. Real-World Example Values

### Arm Mechanism (Encoder ticks)
```java
// Example for arm rotating ~180 degrees (8192 ticks)
pidController = new PIDController(0.0005, 0.000001, 0.0001);
pidController.setTolerance(50); // 50 encoder ticks tolerance
```

### Elevator (Inches)
```java
// Example for 24-inch travel elevator
pidController = new PIDController(0.2, 0.0, 0.05);
pidController.setTolerance(0.1); // 0.1 inch tolerance
```

### Remember: These are starting points! Every mechanism is different.

---

## 6. Essential Debugging Tips

### Monitor Your Values
```java
@Override
public void periodic() {
    SmartDashboard.putNumber("Arm Position", encoder.getPosition());
    SmartDashboard.putNumber("Arm Target", pidController.getSetpoint());
    SmartDashboard.putNumber("Arm Error", pidController.getPositionError());
    SmartDashboard.putNumber("Arm Output", lastPIDOutput);
}
```

### Common Problems & Solutions

**Problem:** System doesn't move at all
- **Solution:** Increase kP significantly

**Problem:** Oscillates wildly around target  
- **Solution:** Reduce kP, add kD

**Problem:** Moves toward target but stops short
- **Solution:** Add small kI, or increase kP slightly

**Problem:** Overshoots and takes forever to settle
- **Solution:** Increase kD

**Problem:** Motor saturates (always at 100% power)
- **Solution:** Much smaller kP, check if target is reachable

---

## 7. Advanced Tips

### Handling Gravity (Feedforward)
```java
// For arms that fight gravity
double gravityCompensation = Math.cos(Math.toRadians(armAngle)) * 0.1;
double pidOutput = pidController.calculate(current, target);
motor.set(pidOutput + gravityCompensation);
```

### Output Limiting
```java
// Prevent violent movements
double output = pidController.calculate(current, target);
output = MathUtil.clamp(output, -0.5, 0.5); // Limit to ±50% power
motor.set(output);
```

### Multiple PID Slots
```java
// Different gains for different situations
if (holdingGamePiece) {
    pidController.setPID(0.1, 0.001, 0.02); // More aggressive
} else {
    pidController.setPID(0.05, 0.0, 0.01);  // Gentler
}
```

---

## 8. Practice Project

**Build this step-by-step:**

1. **Basic P Control** - Get mechanism moving to approximate position
2. **Add D Control** - Smooth out the movement
3. **Add I Control** - Fine-tune for precision (if needed)
4. **Add Feedforward** - Compensate for gravity/friction
5. **Integrate Commands** - Create reusable position commands

**Success criteria:**
- Reaches target within tolerance consistently
- Movement looks smooth and controlled
- Responds well to different target positions

---

## Where to Go Next

**Ready for advanced control? Explore these:**

**🎯 Advanced PID Features**
- [WPILib PID Controller](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html) - Complete API reference
- [Profiled PID](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/profiled-pidcontroller.html) - Velocity and acceleration limits

**🔧 System Identification**
- [SysId Tool](https://docs.wpilib.org/en/stable/docs/software/pathplanning/system-identification/index.html) - Mathematically derive optimal gains
- [Feedforward Control](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html) - Physics-based control

**📊 State-Space Control**
- [Linear Quadratic Regulator](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html) - Optimal control theory
- [Kalman Filters](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html) - Sensor fusion

**⚡ Motor Controller PID**
- [Phoenix 6 PID](https://v6.docs.ctr-electronics.com/) - On-board PID control
- [REV PID](https://docs.revrobotics.com/) - SparkMax closed-loop control

---

**🚀 Ready to start tuning?** Work through the practice project with a simple mechanism, then apply these concepts to arms, elevators, shooters, and more complex systems!


**Next steps:** Advanced control theory and system identification using [WPILib's PID documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/index.html).