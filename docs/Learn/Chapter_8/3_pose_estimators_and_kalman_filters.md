# Pose Estimation: Getting started

## Overview

This section covers the fundementals behind Pose Estimation and all the content needed to understand what's going on "under the hood." Understanding how kalman filters blend vision and wheel odometry together will greatly help users program accurate compeititon ready pose estimation for autonomous trajectory and robot automation.

>**Important:** Before going over this section, ensure that you've read through WPILIB coordinate system(TODO put link here). Common Pose Estimation issues arise when the user doesn't understand how the WPILIB coordinate system works, causing them to add negatives to coordinates that shouldn't be inverted or placing their pose on the wrong alliance, etc. etc. 

>**Important** This section also contains example code related to [Swerve Drive Control](../swerve_control.md). It is recommended to review that first before going through this section.

**What you'll build:** A robot that maintains accurate position tracking by fusing encoder, gyro, and vision measurements.

**Next steps:** Advanced sensor fusion, custom state-space models, and competition-ready localization using [WPILib's Advanced Controls](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/index.html).

---

## 1. Why Kalman Filters?

### The Sensor Fusion Problem

Illustraed below are the 3 inputs the user can take for defining a robot position on the field and their benifits/disadvantages.

### Wheel Odometry

Wheel odometry provides accurate short-term position tracking but can drift over time due to wheel slip.

```java
Pose2d wheelPose = odometry.getPoseMeters();
```

### Vision

Vision systems offer accurate measurements but are often noisy and intermittent, leading to jumpy or occasionally incorrect readings.

```java
Pose2d visionPose = camera.getRobotPose();
```

### Gyro

Gyros are reliable for tracking rotation but may slowly drift over time.

```java
Rotation2d heading = gyro.getRotation2d();
```
Solving this issue requires the use of a Kalman filter, which can seamlessly combine these three sources of data to produce a pose that the user can trust with high precision and accuracy.

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

In a over oversimplified sense, Kalman filters work in a Two Step Model shown below

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

For more infomration on this subject, please check out [Kalman filter theory and background](https://www.kalmanfilter.net/background.html)

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
VecBuilder.fill(0.01)  // Larger = trust measurements more, trust model less
```

The values that are shoved in are refered to as **Standard Deviations**. For purposes of this section, you can think of them as simply inverted "confidence values" for your kalman model.

In a more deep explanation, standard deviation is a measure of uncertainty or variability in your model or sensor readings. It quantifies how much you expect your values to deviate from the true value.

>**Note:** Please note the difference between **Model** and **Measurment** standard deviations as shown above. However, also note that WPILIB pose estimation only uses the latter "Measurement" standard deviation.




**Standard Deviation General Guidelines:**
- **Noisy measurment?** Increase measurement uncertainty (0.01 → 0.1)
- **Model inaccurate?** Increase model uncertainty (3.0 → 10.0)
- **Want faster response?** Decrease measurement uncertainty
- **Want smoother output?** Increase measurement uncertainty

---

## 4. WPILib Pose Estimators

WPILIB makes pose estimation easy by combining data from your gyro, vision system, and wheel odometry into a single library object. This means you don’t have to manually blend sensor readings or build your own Kalman filter—WPILIB does it all for you, giving you smooth and accurate robot position tracking with just a few lines of code. 

Below is an example on how to use a pose estimator within a Swerve Drive implementation. Notice the 2 lines of **Measurment Standard Deviation** in the parameters.

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
            

            //NOTE: These stantdard deviations are provided as an example, std devs will differ depending on the team, vision setup, and preferences.
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
In simple terms, many 3rd party vision vendors have provided libraries that can take the apriltage information collected and process it into a "Pose2d" object through triangulation. It is recommended to use one of the already existing libraries to obtain a pose through their already tested Localization Piplines

We will use the [PhotonVision](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html). implementation of Vision provided Pose as it is the easiest to explain. Examples from Limelight can be found at their doucmentation at
 [Limelight](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2).
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

## 5. Tuning Your WPILIB Standard Deviations

WPILIB pose estimator takes in two types of Measurement Standard Deviations: "Odometry/Gyro", and "Vision".

Teams have different preferences for tuning their numbers. Here are a few guidelines however:
- Keep x and y standard deviations consistent within a certain Standard Deviation set as shown below(AKA don't make your x uncertainty and y uncertainty different values)
- Keep a higher trust on Odometry(Lower Number) and lower trust on vision(Higher Number)to prevent noisy pose estimation (Remember, odometry is smooth and can bridge the gaps between vision estimations)
- Contrary to the previous statment made that rotation drifts over time, many teams recommend putting higher trust values on gyro data as it has proven to be accurate enough over a match period without the need for vision correction.

### Understanding Standard Deviations
```java
// Odometry standard deviations - how much we trust odometry
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

Below are some preset Measurement Standard Deviations you can copy over to your pose estimation enviroment:

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
The concept for this is based on the fact vision becomes more accurate, precise, and less noisy the closer you are to the tag. This logic scales the vision standard deviation to match how far the robot is away from the tag. Farther = less reliable.

For a real-world example of distance-based vision tuning, see [Mechanical Advantage's Vision.java (line 212)](https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/850a1f7656bf3c4603d671caabb7947fcaa0f262/src/main/java/org/littletonrobotics/frc2025/subsystems/vision/Vision.java#L212), where the vision measurement standard deviation is dynamically scaled based on the robot's distance to the AprilTag.
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

For those that want to get into automated scoring features, a simple "pose to pose" code can be a great jumping off point. 

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

---

## 7. Debugging and Monitoring

### Essential Telemetry

It is recommended to log every component (Vision, Gyro, Wheel Odometry Poses) to better debug if your Pose Estimator is not throwing the right positioning. Below is a copiable example of what to log for debugging:

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

Below is a simple code sinpet that can test how accurate your vision system is for pose estimation. This can help determine standard deviations or camera placment.

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

**Explore these topics related to Pose Estimation:**

> **TODO:** Redo the links list below to include relevant resources for pose estimation, Kalman filters, WPILib documentation, and vision integration. Ensure all links are up-to-date and clearly labeled for easy navigation.

---

