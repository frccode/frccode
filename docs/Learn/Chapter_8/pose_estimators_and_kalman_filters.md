# Kalman Filters & Pose Estimation: Getting Started

## Overview

Kalman filters are "smart" sensors that combine multiple imperfect measurements to create a better estimate than any single sensor alone. Think of them as a mathematical way to blend wheel odometry, gyroscopes, and vision data to know exactly where your robot is on the field - even when individual sensors drift or have noise.

**What you'll build:** A robot that maintains accurate position tracking by fusing encoder, gyro, and vision measurements.

**Next steps:** Advanced sensor fusion, custom state-space models, and competition-ready localization using [WPILib's Advanced Controls](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/index.html).

---

## 1. Why Kalman Filters?

### The Sensor Fusion Problem
**Individual sensors have problems:**
```java
// Wheel odometry: Accurate short-term, drifts over time
Pose2d wheelPose = odometry.getPoseMeters(); // Drifts due to wheel slip

// Vision: Accurate but noisy and intermittent  
Pose2d visionPose = camera.getRobotPose(); // Jumpy, sometimes wrong

// Gyro: Good for rotation, but can drift
Rotation2d heading = gyro.getRotation2d(); // Slowly drifts over time
```

**Kalman filter solution:**
```java
// Fuses all sensors for best estimate
poseEstimator.update(gyro.getRotation2d(), leftDistance, rightDistance);
poseEstimator.addVisionMeasurement(visionPose, timestamp);
Pose2d bestEstimate = poseEstimator.getEstimatedPosition(); // Smooth and accurate!
```

**Real Benefits:**
- **Smooth tracking** - No jumpy position updates
- **Drift correction** - Vision fixes wheel/gyro drift
- **Noise rejection** - Filters out sensor noise automatically
- **Confidence weighting** - Trusts reliable sensors more
- **Latency compensation** - Handles delayed vision measurements

---

## 2. How Kalman Filters Work

### The Two-Step Dance

**Step 1: Predict** (What we think happened)
```
Previous State + Model + Time = Predicted State
```

**Step 2: Correct** (Update with measurements)
```
Predicted State + New Measurement + Confidence = Updated State
```

### Simple Example: Robot Position
```java
// Step 1: Predict where robot moved based on wheels
predictedX = lastX + (leftWheel + rightWheel) / 2 * cos(heading);
predictedY = lastY + (leftWheel + rightWheel) / 2 * sin(heading);

// Step 2: Correct with vision (if available)
if (hasVisionMeasurement) {
    // Blend prediction with vision based on confidence
    finalX = blendWithConfidence(predictedX, visionX, confidence);
    finalY = blendWithConfidence(predictedY, visionY, confidence);
}
```

### Key Concepts

**State** - What we're tracking (robot X, Y, rotation)
**Model** - How we predict state changes (wheel movement)
**Measurement** - What sensors tell us (vision pose)
**Uncertainty** - How much we trust each source
**Covariance** - Mathematical measure of uncertainty

---

## 3. Basic Kalman Filter Implementation

### Simple Flywheel Example
```java
public class SmartFlywheelSubsystem extends SubsystemBase {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    
    // The system model (how flywheel behaves)
    private final LinearSystem<N1, N1, N1> flywheelPlant;
    
    // The Kalman filter
    private final KalmanFilter<N1, N1, N1> kalmanFilter;
    
    public SmartFlywheelSubsystem() {
        motor = new CANSparkMax(1, MotorType.kBrushless);
        encoder = motor.getEncoder();
        
        // Create system model from characterization
        double kV = 0.023; // Volts per (rad/s)  
        double kA = 0.001; // Volts per (rad/s²)
        flywheelPlant = LinearSystemId.identifyVelocitySystem(kV, kA);
        
        // Create Kalman filter
        kalmanFilter = new KalmanFilter<>(
            Nat.N1(), Nat.N1(),
            flywheelPlant,
            VecBuilder.fill(3.0),    // Model uncertainty (trust model 3 rad/s)
            VecBuilder.fill(0.01),   // Measurement uncertainty (trust encoder 0.01 rad/s)
            0.020                    // Loop time (20ms)
        );
    }
    
    public void setVoltage(double volts) {
        // Update the filter with our control input
        kalmanFilter.predict(VecBuilder.fill(volts), 0.020);
        motor.setVoltage(volts);
    }
    
    @Override
    public void periodic() {
        // Update filter with encoder measurement
        double encoderVelocity = encoder.getVelocity() * 2 * Math.PI / 60; // Convert RPM to rad/s
        kalmanFilter.correct(VecBuilder.fill(volts), VecBuilder.fill(encoderVelocity));
        
        // Get the filtered estimate
        double filteredVelocity = kalmanFilter.getXhat(0);
        SmartDashboard.putNumber("Raw Encoder", encoderVelocity);
        SmartDashboard.putNumber("Filtered Velocity", filteredVelocity);
    }
    
    public double getFilteredVelocity() {
        return kalmanFilter.getXhat(0);
    }
}
```

### Understanding the Parameters
```java
// Model uncertainty: How much we trust our physics model
VecBuilder.fill(3.0)   // Larger = trust model less, trust measurements more

// Measurement uncertainty: How much we trust our sensors  
VecBuilder.fill(0.01)  // Larger = trust measurements less, trust model more
```

**Tuning Rules:**
- **Noisy encoder?** Increase measurement uncertainty (0.01 → 0.1)
- **Model inaccurate?** Increase model uncertainty (3.0 → 10.0)
- **Want faster response?** Decrease measurement uncertainty
- **Want smoother output?** Increase measurement uncertainty

---

## 4. WPILib Pose Estimators

### Swerve Drive Pose Estimator
```java
public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveDriveKinematics kinematics;
    private final Gyro gyro;
    private final SwerveModule[] modules;
    
    // The pose estimator (Kalman filter for robot position)
    private final SwerveDrivePoseEstimator poseEstimator;
    
    public SwerveDriveSubsystem() {
        gyro = new Pigeon2(1);
        modules = new SwerveModule[4]; // Initialize your modules
        
        // Create kinematics
        kinematics = new SwerveDriveKinematics(
            new Translation2d(0.3, 0.3),   // Front left
            new Translation2d(0.3, -0.3),  // Front right  
            new Translation2d(-0.3, 0.3),  // Back left
            new Translation2d(-0.3, -0.3)  // Back right
        );
        
        // Create pose estimator
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            gyro.getRotation2d(),
            getModulePositions(),
            new Pose2d(),
            
            // State standard deviations [x, y, rotation]
            VecBuilder.fill(0.1, 0.1, Math.toRadians(5)),  // Trust odometry
            
            // Vision standard deviations [x, y, rotation]  
            VecBuilder.fill(0.5, 0.5, Math.toRadians(30))  // Less trust in vision
        );
    }
    
    @Override
    public void periodic() {
        // Update with odometry every loop (fast, reliable)
        poseEstimator.update(
            gyro.getRotation2d(),
            getModulePositions()
        );
        
        // Add vision measurements when available (slower, accurate)
        Optional<Pose2d> visionPose = getVisionPose();
        if (visionPose.isPresent()) {
            poseEstimator.addVisionMeasurement(
                visionPose.get(),
                Timer.getFPGATimestamp() - 0.1  // Account for camera latency
            );
        }
        
        // Send to dashboard
        Pose2d currentPose = poseEstimator.getEstimatedPosition();
        SmartDashboard.putNumber("Robot X", currentPose.getX());
        SmartDashboard.putNumber("Robot Y", currentPose.getY());
        SmartDashboard.putNumber("Robot Rotation", currentPose.getRotation().getDegrees());
    }
    
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
    
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(
            gyro.getRotation2d(),
            getModulePositions(),
            pose
        );
    }
    
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }
}
```

### Vision Integration
```java
public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final AprilTagFieldLayout fieldLayout;
    
    public VisionSubsystem() {
        camera = new PhotonCamera("main_camera");
        
        // Load field layout
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(
                AprilTagFields.k2024Crescendo.m_resourceFile
            );
        } catch (IOException e) {
            throw new RuntimeException("Failed to load AprilTag layout", e);
        }
    }
    
    public Optional<EstimatedRobotPose> getEstimatedPose(Pose2d prevPose) {
        PhotonPipelineResult result = camera.getLatestResult();
        
        if (!result.hasTargets()) {
            return Optional.empty();
        }
        
        // Use PhotonPoseEstimator for 3D pose estimation
        PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            robotToCamera // Transform from robot center to camera
        );
        
        return poseEstimator.update();
    }
}
```

---

## 5. Tuning Your Pose Estimator

### Understanding Standard Deviations
```java
// State standard deviations - how much we trust odometry
VecBuilder.fill(
    0.1,                     // X uncertainty: ±10cm
    0.1,                     // Y uncertainty: ±10cm  
    Math.toRadians(5)        // Rotation uncertainty: ±5°
);

// Vision standard deviations - how much we trust vision
VecBuilder.fill(
    0.5,                     // X uncertainty: ±50cm
    0.5,                     // Y uncertainty: ±50cm
    Math.toRadians(30)       // Rotation uncertainty: ±30°
);
```

### Tuning Guidelines

**Good Odometry (new wheels, accurate gyro):**
```java
// Trust odometry more
VecBuilder.fill(0.05, 0.05, Math.toRadians(2))  // State std devs
VecBuilder.fill(0.8, 0.8, Math.toRadians(45))   // Vision std devs
```

**Poor Odometry (worn wheels, drifting gyro):**
```java
// Trust vision more
VecBuilder.fill(0.2, 0.2, Math.toRadians(10))   // State std devs
VecBuilder.fill(0.3, 0.3, Math.toRadians(15))   // Vision std devs
```

**Distance-Based Vision Tuning:**
```java
public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    // Calculate distance to nearest AprilTag
    double distance = calculateDistanceToTag(visionPose);
    
    // Scale uncertainty by distance (farther = less reliable)
    double baseStdDev = 0.1;
    double scaledStdDev = baseStdDev * (1.0 + distance / 5.0);
    
    poseEstimator.addVisionMeasurement(
        visionPose,
        timestamp,
        VecBuilder.fill(scaledStdDev, scaledStdDev, Math.toRadians(30))
    );
}
```

---

## 6. Real-World Applications

### Auto-Alignment with Pose Estimation
```java
public class DriveToPositionCommand extends CommandBase {
    private final SwerveDriveSubsystem drive;
    private final Pose2d targetPose;
    private final PIDController xController = new PIDController(2.0, 0.0, 0.1);
    private final PIDController yController = new PIDController(2.0, 0.0, 0.1);
    private final PIDController rotController = new PIDController(3.0, 0.0, 0.1);
    
    public DriveToPositionCommand(SwerveDriveSubsystem drive, Pose2d target) {
        this.drive = drive;
        this.targetPose = target;
        addRequirements(drive);
        
        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    @Override
    public void execute() {
        // Get current pose from estimator (fused odometry + vision)
        Pose2d currentPose = drive.getPose();
        
        // Calculate control outputs
        double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
        double rotSpeed = rotController.calculate(
            currentPose.getRotation().getRadians(),
            targetPose.getRotation().getRadians()
        );
        
        // Drive robot
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        drive.drive(speeds);
    }
    
    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && 
               yController.atSetpoint() && 
               rotController.atSetpoint();
    }
}
```

### Field-Relative Autonomous
```java
public class FieldRelativeAuto extends SequentialCommandGroup {
    public FieldRelativeAuto(SwerveDriveSubsystem drive) {
        addCommands(
            // Drive to specific field positions using pose estimation
            new DriveToPositionCommand(drive, new Pose2d(2, 1, Rotation2d.fromDegrees(0))),
            new DriveToPositionCommand(drive, new Pose2d(4, 3, Rotation2d.fromDegrees(180))),
            new DriveToPositionCommand(drive, new Pose2d(1, 5, Rotation2d.fromDegrees(90)))
        );
    }
}
```

---

## 7. Debugging and Monitoring

### Essential Telemetry
```java
@Override
public void periodic() {
    // Pose estimator state
    Pose2d pose = poseEstimator.getEstimatedPosition();
    SmartDashboard.putNumber("Pose X", pose.getX());
    SmartDashboard.putNumber("Pose Y", pose.getY());
    SmartDashboard.putNumber("Pose Rotation", pose.getRotation().getDegrees());
    
    // Individual sensor readings
    SmartDashboard.putNumber("Gyro Angle", gyro.getRotation2d().getDegrees());
    SmartDashboard.putNumber("Left Distance", leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Distance", rightEncoder.getDistance());
    
    // Vision data
    Optional<Pose2d> visionPose = getVisionPose();
    SmartDashboard.putBoolean("Has Vision", visionPose.isPresent());
    if (visionPose.isPresent()) {
        SmartDashboard.putNumber("Vision X", visionPose.get().getX());
        SmartDashboard.putNumber("Vision Y", visionPose.get().getY());
    }
    
    // Uncertainty monitoring (advanced)
    Matrix<N3, N3> covariance = poseEstimator.getCovariance();
    SmartDashboard.putNumber("X Uncertainty", Math.sqrt(covariance.get(0, 0)));
    SmartDashboard.putNumber("Y Uncertainty", Math.sqrt(covariance.get(1, 1)));
}
```

### Common Issues & Solutions

**Problem:** Position jumps when vision appears
- **Solution:** Increase vision standard deviations, check camera calibration

**Problem:** Pose drifts even with vision
- **Solution:** Decrease vision standard deviations, check AprilTag positions

**Problem:** Jerky movement during auto-alignment
- **Solution:** Increase state standard deviations, tune PID controllers

**Problem:** Vision measurements rejected
- **Solution:** Check timestamp calculation, verify field layout loading

**Problem:** Poor performance/lag
- **Solution:** Reduce vision frequency, optimize pose calculation

### Quality Metrics
```java
public class PoseEstimatorDiagnostics {
    private final List<Double> visionErrors = new ArrayList<>();
    
    public void checkVisionQuality(Pose2d odometryPose, Pose2d visionPose) {
        double error = odometryPose.getTranslation()
                                  .getDistance(visionPose.getTranslation());
        visionErrors.add(error);
        
        // Keep last 10 measurements
        if (visionErrors.size() > 10) {
            visionErrors.remove(0);
        }
        
        // Calculate average error
        double avgError = visionErrors.stream().mapToDouble(Double::doubleValue).average().orElse(0);
        SmartDashboard.putNumber("Avg Vision Error", avgError);
        
        // Warn if vision quality is poor
        if (avgError > 0.5) { // 50cm average error
            DriverStation.reportWarning("Poor vision quality detected", false);
        }
    }
}
```

---

## 8. Practice Project

**Build this step-by-step:**

1. **Basic Kalman Filter** - Implement simple flywheel velocity filtering
2. **Pose Estimator Setup** - Create swerve drive pose estimator
3. **Vision Integration** - Add AprilTag pose measurements
4. **Parameter Tuning** - Optimize standard deviations for your robot
5. **Auto-Alignment** - Use fused pose for precise positioning
6. **Competition Integration** - Build reliable autonomous routines

**Success Criteria:**
- Smooth position tracking without jumps
- Vision corrects odometry drift over time
- Accurate positioning within 5cm after vision correction
- Reliable autonomous navigation

---

## Where to Go Next

**Ready for advanced estimation? Explore these:**

**🎯 Advanced State-Space Control**
- [Linear Quadratic Regulator](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html#the-linear-quadratic-regulator) - Optimal feedback control
- [Custom State-Space Models](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html) - Model complex mechanisms

**🔧 Sensor Fusion Techniques**
- [Unscented Kalman Filters](https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/estimator/UnscentedKalmanFilter.java) - Handle nonlinear systems
- [Multi-Camera Fusion](https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html) - Combine multiple vision sources

**📊 Performance Optimization**
- [Extended Kalman Filters](https://en.wikipedia.org/wiki/Extended_Kalman_filter) - Advanced nonlinear estimation
- [Particle Filters](https://en.wikipedia.org/wiki/Particle_filter) - Handle non-Gaussian noise

**⚡ Competition Applications**
- [PathPlanner Integration](https://pathplanner.dev/pplib-getting-started.html) - Pose-corrected path following
- [Real-Time Trajectory Correction](https://docs.wpilib.org/en/stable/docs/software/pathplanning/index.html) - Adaptive autonomous
- [Match Replay Analysis](https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html) - Post-match pose tracking

---

**🚀 Ready to master robot localization?** Start with basic filtering, understand the fundamentals, then build competition-ready pose estimation systems that rival GPS accuracy!