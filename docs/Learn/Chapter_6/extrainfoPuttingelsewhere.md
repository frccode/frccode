## 8. Advanced Techniques

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

## 9. Practice Project

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
```java
// Start with aggressive P terms
xController.setP(2.0);
yController.setP(2.0);
rotationController.setP(1.0);

// Increase until jitter occurs during path following
// Back off to stable values
// Tune BEFORE adding vision - vision can introduce oscillations
```