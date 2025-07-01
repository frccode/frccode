# AprilTag Processing: Getting Started

## Overview

AprilTags are like QR codes for robots - fiducial markers that provide precise position and orientation data. Unlike basic vision targets, AprilTags tell your robot exactly where it is on the field and can enable centimeter-accurate autonomous navigation and positioning.

**What you'll build:** A robot that uses AprilTags for precise field localization and automated alignment.

**Next steps:** Multi-tag pose estimation, camera calibration, and integration with advanced odometry using [PhotonVision](https://docs.photonvision.org/) and [WPILib Pose Estimation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html).

---

## 1. What Are AprilTags?

### The Smart Fiducial Marker
**AprilTag vs Regular Vision Target:**
```java
// Regular target: Only tells you angle/distance
VisionTarget reflectiveTape = findGreenRectangle();
double angle = calculateAngle(reflectiveTape);

// AprilTag: Tells you exact position and orientation
AprilTagDetection tag = detector.detect(image);
Pose3d robotPose = tag.getPose();  // Full 6DOF position!
```

**Key Features:**
- **Unique IDs** - Each tag has a distinct pattern (587 possible in 36h11 family)
- **6DOF Pose** - X, Y, Z position + roll, pitch, yaw rotation
- **Robust Detection** - Works with partial occlusion and poor lighting
- **No Calibration Required** - For basic centering applications

### FRC 2023+ Implementation
**Field Layout:**
- Multiple tags placed at known positions throughout the field
- Each tag is 6.5" × 6.5" with unique ID
- Tags mounted flat on field elements
- Standard 36h11 family used

**Applications:**
- **Auto-alignment** - Drive until tag is centered
- **Pose estimation** - Know exact robot position
- **Autonomous navigation** - Navigate using field landmarks
- **Backup odometry** - Correct wheel/gyro drift

---

## 2. How AprilTag Detection Works

### The 8-Step Detection Pipeline

**Step 1: Grayscale Conversion**
```
Color Image → Grayscale Image
(Remove color, keep brightness only)
```

**Step 2: Image Downsampling**
```
High Resolution → Lower Resolution
(Faster processing, less precision)
```

**Step 3: Adaptive Thresholding**
```
Grayscale → Binary (Black/White)
(Each pixel classified as light/dark/unsure)
```

**Step 4: Connected Components**
```
Binary Image → Pixel Clusters
(Group connected pixels, discard small clusters)
```

**Step 5: Quadrilateral Fitting**
```
Pixel Clusters → 4-Sided Shapes
(Find best-fit rectangles for each cluster)
```

**Step 6: Tag Candidate Selection**
```
All Quadrilaterals → Potential Tags
(Look for nested quad pattern: outer border + inner grid)
```

**Step 7: Bit Pattern Decoding**
```
Tag Candidate → Tag ID
(Read 6×6 grid pattern, match to known tag)
```

**Step 8: Sub-pixel Refinement**
```
Low-res Detection → High-precision Corners
(Re-analyze at full resolution for accuracy)
```

### Simplified Pipeline Visualization
```
Camera Image
     ↓
[Grayscale + Downsample]
     ↓
[Find Black/White Regions]
     ↓
[Fit Rectangles to Regions]
     ↓
[Decode Interior Pattern]
     ↓
[Refine Corner Positions]
     ↓
AprilTag Detection (ID + Pose)
```

---

## 3. Basic Implementation

### Step 1: Simple Tag Detection
```java
public class AprilTagVision extends SubsystemBase {
    private final UsbCamera camera;
    private final AprilTagDetector detector;
    private final CvSink cvSink;
    private final Mat frame = new Mat();
    
    private AprilTagDetection[] lastDetections = new AprilTagDetection[0];
    
    public AprilTagVision() {
        // Camera setup
        camera = CameraServer.startAutomaticCapture();
        camera.setResolution(640, 480);
        camera.setFPS(30);
        
        cvSink = CameraServer.getVideo();
        
        // AprilTag detector setup
        detector = new AprilTagDetector();
        detector.addFamily(AprilTagFamily.TAG_36h11); // FRC standard
        
        // Configure detection parameters
        AprilTagDetector.Config config = detector.getConfig();
        config.decimationFactor = 2;  // Downsample by factor of 2
        config.numThreads = 2;        // Use 2 CPU cores
        detector.setConfig(config);
    }
    
    @Override
    public void periodic() {
        // Grab frame
        if (cvSink.grabFrame(frame) == 0) {
            return; // No new frame
        }
        
        // Detect AprilTags
        lastDetections = detector.detect(frame);
        
        // Send results to dashboard
        SmartDashboard.putNumber("Tags Detected", lastDetections.length);
        
        for (int i = 0; i < lastDetections.length; i++) {
            AprilTagDetection detection = lastDetections[i];
            SmartDashboard.putNumber("Tag " + i + " ID", detection.getId());
            SmartDashboard.putNumber("Tag " + i + " Center X", detection.getCenterX());
            SmartDashboard.putNumber("Tag " + i + " Center Y", detection.getCenterY());
        }
    }
    
    public AprilTagDetection[] getDetections() {
        return lastDetections;
    }
    
    public boolean hasTag(int targetID) {
        for (AprilTagDetection detection : lastDetections) {
            if (detection.getId() == targetID) {
                return true;
            }
        }
        return false;
    }
    
    public AprilTagDetection getTag(int targetID) {
        for (AprilTagDetection detection : lastDetections) {
            if (detection.getId() == targetID) {
                return detection;
            }
        }
        return null;
    }
}
```

### Step 2: Auto-Alignment (Simple)
```java
public class AlignToTagCommand extends CommandBase {
    private final SwerveDrive drive;
    private final AprilTagVision vision;
    private final int targetTagID;
    private final PIDController rotationController;
    
    public AlignToTagCommand(SwerveDrive drive, AprilTagVision vision, int tagID) {
        this.drive = drive;
        this.vision = vision;
        this.targetTagID = tagID;
        this.rotationController = new PIDController(0.1, 0.0, 0.01);
        
        addRequirements(drive);
    }
    
    @Override
    public void execute() {
        AprilTagDetection tag = vision.getTag(targetTagID);
        
        if (tag != null) {
            // Calculate error from center of image
            double imageWidth = 640;
            double centerX = imageWidth / 2.0;
            double pixelError = tag.getCenterX() - centerX;
            
            // Convert to angular error (rough approximation)
            double angularError = pixelError / centerX; // Normalized error
            
            // Use PID to control rotation
            double rotationSpeed = rotationController.calculate(0, angularError);
            
            // Drive with rotation correction
            ChassisSpeeds speeds = new ChassisSpeeds(0, 0, rotationSpeed);
            drive.drive(speeds);
        } else {
            // No tag visible, stop
            drive.drive(new ChassisSpeeds());
        }
    }
    
    @Override
    public boolean isFinished() {
        AprilTagDetection tag = vision.getTag(targetTagID);
        if (tag == null) return false;
        
        double imageWidth = 640;
        double centerX = imageWidth / 2.0;
        double pixelError = Math.abs(tag.getCenterX() - centerX);
        
        return pixelError < 10; // Within 10 pixels of center
    }
}
```

---

## 4. Advanced Pose Estimation

### Camera Calibration and 3D Pose
```java
public class AdvancedAprilTagVision extends SubsystemBase {
    private final AprilTagDetector detector;
    private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator poseEstimator;
    
    // Camera calibration parameters (get these from camera calibration)
    private final Matrix<N3, N3> cameraMatrix = MatBuilder.fill(Nat.N3(), Nat.N3(),
        500.0, 0.0, 320.0,     // fx, 0, cx
        0.0, 500.0, 240.0,     // 0, fy, cy  
        0.0, 0.0, 1.0          // 0, 0, 1
    );
    
    private final Matrix<N5, N1> distortionCoefficients = MatBuilder.fill(Nat.N5(), Nat.N1(),
        0.1, -0.2, 0.0, 0.0, 0.0  // k1, k2, p1, p2, k3
    );
    
    public AdvancedAprilTagVision() {
        detector = new AprilTagDetector();
        detector.addFamily(AprilTagFamily.TAG_36h11);
        
        // Load field layout (tag positions)
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(
                AprilTagFields.k2024Crescendo.m_resourceFile
            );
        } catch (IOException e) {
            throw new RuntimeException("Failed to load AprilTag field layout", e);
        }
        
        // Create pose estimator
        Transform3d robotToCamera = new Transform3d(
            new Translation3d(0.0, 0.0, 0.5), // Camera 0.5m above robot center
            new Rotation3d(0, 0, 0)            // Camera facing forward
        );
        
        poseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera
        );
    }
    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        // Convert AprilTag detections to PhotonVision format
        List<PhotonTrackedTarget> targets = new ArrayList<>();
        
        for (AprilTagDetection detection : getDetections()) {
            // Calculate 3D pose of tag relative to camera
            Pose3d tagPose = calculateTagPose(detection);
            
            PhotonTrackedTarget target = new PhotonTrackedTarget(
                Math.atan2(detection.getCenterY() - 240, 500), // yaw
                Math.atan2(detection.getCenterX() - 320, 500), // pitch  
                0.0,                                           // area (not used)
                0.0,                                           // skew (not used)
                detection.getId(),                             // fiducial ID
                tagPose,                                       // best camera to target
                tagPose,                                       // alternate camera to target
                0.0,                                           // pose ambiguity
                List.of(),                                     // corners (empty for now)
                List.of()                                      // detected corners (empty)
            );
            
            targets.add(target);
        }
        
        // Create PhotonPipelineResult
        PhotonPipelineResult result = new PhotonPipelineResult(
            Timer.getFPGATimestamp(),
            targets
        );
        
        return poseEstimator.update(result);
    }
    
    private Pose3d calculateTagPose(AprilTagDetection detection) {
        // Use PnP (Perspective-n-Point) algorithm to get 3D pose
        // This requires camera calibration for accuracy
        
        // Tag corners in 3D (6.5" square tag)
        double tagSize = 0.1651; // 6.5 inches in meters
        double half = tagSize / 2.0;
        
        List<Translation3d> objectPoints = List.of(
            new Translation3d(-half, -half, 0),
            new Translation3d(half, -half, 0),
            new Translation3d(half, half, 0),
            new Translation3d(-half, half, 0)
        );
        
        // Detected corners in image
        CornerArray corners = detection.getCorners();
        List<Translation2d> imagePoints = List.of(
            new Translation2d(corners.get(0).x, corners.get(0).y),
            new Translation2d(corners.get(1).x, corners.get(1).y),
            new Translation2d(corners.get(2).x, corners.get(2).y),
            new Translation2d(corners.get(3).x, corners.get(3).y)
        );
        
        // Solve PnP to get pose (simplified - real implementation more complex)
        return OpenCVHelp.solvePNP_SQUARE(
            cameraMatrix, distortionCoefficients, 
            objectPoints, imagePoints
        );
    }
}
```

### Integration with Odometry
```java
public class DriveSubsystem extends SubsystemBase {
    private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final AdvancedAprilTagVision vision;
    
    public DriveSubsystem(AdvancedAprilTagVision vision) {
        this.vision = vision;
        
        // Create pose estimator that fuses wheel odometry with vision
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getGyroRotation(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1),    // Odometry standard deviations
            VecBuilder.fill(0.5, 0.5, 0.5)     // Vision standard deviations
        );
    }
    
    @Override
    public void periodic() {
        // Update odometry with wheel and gyro data
        poseEstimator.update(getGyroRotation(), getModulePositions());
        
        // Add vision measurements
        Optional<EstimatedRobotPose> visionPose = vision.getEstimatedGlobalPose();
        if (visionPose.isPresent()) {
            EstimatedRobotPose estimate = visionPose.get();
            poseEstimator.addVisionMeasurement(
                estimate.estimatedPose.toPose2d(),
                estimate.timestampSeconds
            );
        }
        
        // Send pose to dashboard
        Pose2d currentPose = poseEstimator.getEstimatedPosition();
        SmartDashboard.putNumber("Robot X", currentPose.getX());
        SmartDashboard.putNumber("Robot Y", currentPose.getY());
        SmartDashboard.putNumber("Robot Rotation", currentPose.getRotation().getDegrees());
    }
    
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
    
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
    }
}
```

---

## 5. Real-World Applications

### Auto-Scoring Alignment
```java
public class AutoScoreCommand extends CommandBase {
    private final SwerveDrive drive;
    private final AprilTagVision vision;
    private final int scoringTagID;
    private final double targetDistance; // meters
    
    private final PIDController xController = new PIDController(1.0, 0.0, 0.1);
    private final PIDController yController = new PIDController(1.0, 0.0, 0.1);
    private final PIDController rotController = new PIDController(2.0, 0.0, 0.2);
    
    public AutoScoreCommand(SwerveDrive drive, AprilTagVision vision, int tagID) {
        this.drive = drive;
        this.vision = vision;
        this.scoringTagID = tagID;
        this.targetDistance = 1.0; // 1 meter from tag
        
        addRequirements(drive);
    }
    
    @Override
    public void execute() {
        Optional<EstimatedRobotPose> visionPose = vision.getEstimatedGlobalPose();
        
        if (visionPose.isEmpty()) {
            drive.drive(new ChassisSpeeds());
            return;
        }
        
        // Get current robot pose
        Pose2d robotPose = visionPose.get().estimatedPose.toPose2d();
        
        // Get target tag pose from field layout
        Optional<Pose3d> tagPose = vision.getFieldLayout().getTagPose(scoringTagID);
        if (tagPose.isEmpty()) return;
        
        // Calculate desired robot position (in front of tag)
        Pose2d tagPose2d = tagPose.get().toPose2d();
        Translation2d offsetFromTag = new Translation2d(-targetDistance, 0)
            .rotateBy(tagPose2d.getRotation());
        Pose2d targetRobotPose = new Pose2d(
            tagPose2d.getTranslation().plus(offsetFromTag),
            tagPose2d.getRotation().rotateBy(Rotation2d.fromDegrees(180))
        );
        
        // Calculate control outputs
        double xSpeed = xController.calculate(
            robotPose.getX(), 
            targetRobotPose.getX()
        );
        double ySpeed = yController.calculate(
            robotPose.getY(), 
            targetRobotPose.getY()
        );
        double rotSpeed = rotController.calculate(
            robotPose.getRotation().getRadians(),
            targetRobotPose.getRotation().getRadians()
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

### Multi-Tag Localization
```java
public class MultiTagLocalization {
    public static Optional<Pose2d> estimatePoseFromMultipleTags(
            AprilTagDetection[] detections, 
            AprilTagFieldLayout fieldLayout) {
        
        if (detections.length < 2) {
            return Optional.empty(); // Need at least 2 tags for good estimate
        }
        
        List<Pose2d> poseEstimates = new ArrayList<>();
        
        for (AprilTagDetection detection : detections) {
            Optional<Pose3d> tagFieldPose = fieldLayout.getTagPose(detection.getId());
            if (tagFieldPose.isEmpty()) continue;
            
            // Calculate robot pose from this tag
            Pose3d cameraToTag = calculateTagPose(detection);
            Pose3d robotToCamera = new Pose3d(); // Camera at robot center for simplicity
            
            Pose3d fieldToTag = tagFieldPose.get();
            Pose3d fieldToCamera = fieldToTag.transformBy(cameraToTag.inverse());
            Pose3d fieldToRobot = fieldToCamera.transformBy(robotToCamera.inverse());
            
            poseEstimates.add(fieldToRobot.toPose2d());
        }
        
        if (poseEstimates.isEmpty()) {
            return Optional.empty();
        }
        
        // Average all pose estimates (simple fusion)
        double avgX = poseEstimates.stream().mapToDouble(p -> p.getX()).average().orElse(0);
        double avgY = poseEstimates.stream().mapToDouble(p -> p.getY()).average().orElse(0);
        
        // Circular mean for rotation
        double sumSin = poseEstimates.stream().mapToDouble(p -> p.getRotation().getSin()).sum();
        double sumCos = poseEstimates.stream().mapToDouble(p -> p.getRotation().getCos()).sum();
        Rotation2d avgRotation = new Rotation2d(sumCos, sumSin);
        
        return Optional.of(new Pose2d(avgX, avgY, avgRotation));
    }
}
```

---

## 6. Performance Optimization

### Detection Parameters
```java
public void configureForPerformance() {
    AprilTagDetector.Config config = detector.getConfig();
    
    // Fast detection (lower accuracy)
    config.decimationFactor = 4;    // Downsample more aggressively
    config.sigma = 0.8;             // More blur (faster but less precise)
    config.numThreads = 4;          // Use more CPU cores
    config.refineEdges = false;     // Skip sub-pixel refinement
    
    detector.setConfig(config);
}

public void configureForAccuracy() {
    AprilTagDetector.Config config = detector.getConfig();
    
    // Accurate detection (slower)
    config.decimationFactor = 1;    // No downsampling
    config.sigma = 0.0;             // No blur
    config.numThreads = 2;          // Fewer threads to avoid conflicts
    config.refineEdges = true;      // Enable sub-pixel refinement
    
    detector.setConfig(config);
}
```

### Camera Settings for AprilTags
```java
public void optimizeCameraForTags() {
    // Lower resolution for speed (tags don't need high resolution)
    camera.setResolution(320, 240);
    camera.setFPS(30);
    
    // Fixed exposure prevents auto-exposure lag
    camera.setExposure(50);  // Adjust based on field lighting
    
    // Higher contrast helps with black/white detection
    camera.setBrightness(30);
    camera.setContrast(80);
    
    // Disable auto white balance for consistent colors
    camera.setWhiteBalance(3000); // Typical field lighting temperature
}
```

---

## 7. Debugging and Troubleshooting

### Essential Telemetry
```java
@Override
public void periodic() {
    AprilTagDetection[] detections = getDetections();
    
    // Detection info
    SmartDashboard.putNumber("AprilTags Detected", detections.length);
    SmartDashboard.putNumber("Processing Time (ms)", processingTimeMs);
    
    for (int i = 0; i < Math.min(detections.length, 3); i++) {
        AprilTagDetection detection = detections[i];
        String prefix = "Tag" + i + "_";
        
        SmartDashboard.putNumber(prefix + "ID", detection.getId());
        SmartDashboard.putNumber(prefix + "CenterX", detection.getCenterX());
        SmartDashboard.putNumber(prefix + "CenterY", detection.getCenterY());
        SmartDashboard.putNumber(prefix + "Decision_Margin", detection.getDecisionMargin());
        SmartDashboard.putNumber(prefix + "Hamming_Distance", detection.getHammingDistance());
    }
    
    // Pose estimation info
    Optional<EstimatedRobotPose> pose = getEstimatedGlobalPose();
    if (pose.isPresent()) {
        Pose2d robotPose = pose.get().estimatedPose.toPose2d();
        SmartDashboard.putNumber("Vision_X", robotPose.getX());
        SmartDashboard.putNumber("Vision_Y", robotPose.getY());
        SmartDashboard.putNumber("Vision_Rotation", robotPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Vision_Timestamp", pose.get().timestampSeconds);
    }
}
```

### Common Issues & Solutions

**Problem:** No tags detected
- **Solution:** Check lighting, camera exposure, tag visibility and distance

**Problem:** Inconsistent detection
- **Solution:** Improve lighting consistency, check for tag damage/flatness

**Problem:** Poor pose accuracy
- **Solution:** Calibrate camera properly, use multiple tags, check tag mounting

**Problem:** High CPU usage
- **Solution:** Increase decimation factor, reduce threads, lower camera resolution

**Problem:** Pose estimates jumping around
- **Solution:** Filter poses, reject outliers, tune pose estimator standard deviations

### Detection Quality Metrics
```java
public boolean isGoodDetection(AprilTagDetection detection) {
    // Decision margin - how confident the detector is
    // Higher is better (typically > 100 is good)
    if (detection.getDecisionMargin() < 80) {
        return false;
    }
    
    // Hamming distance - number of bit errors corrected
    // Lower is better (0-2 is good, >3 is suspicious)
    if (detection.getHammingDistance() > 2) {
        return false;
    }
    
    // Tag size in image - too small = too far away
    CornerArray corners = detection.getCorners();
    double area = calculatePolygonArea(corners);
    if (area < 100) { // Minimum pixel area
        return false;
    }
    
    return true;
}
```

---

## 8. Practice Project

**Build this step-by-step:**

1. **Basic Detection** - Detect and display AprilTag IDs
2. **Simple Alignment** - Center robot on specific tag
3. **Distance Control** - Stop at specific distance from tag
4. **Pose Estimation** - Calculate robot field position
5. **Multi-Tag Fusion** - Use multiple tags for better accuracy
6. **Autonomous Navigation** - Navigate to scoring positions

**Success Criteria:**
- Detect tags consistently at 3+ meters distance
- Align to tags within 2 degrees accuracy
- Pose estimation within 10cm of actual position
- Smooth autonomous navigation to targets

---

## Where to Go Next

**Ready for production AprilTag systems? Explore these:**

**🎯 Professional Solutions**
- [PhotonVision](https://docs.photonvision.org/) - Complete AprilTag processing pipeline
- [LimelightLib](https://docs.limelightvision.io/) - Hardware-accelerated tag detection
- [WPILib Pose Estimation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html) - Sensor fusion

**🔧 Advanced Techniques**
- [Camera Calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) - Precise 3D pose estimation
- [Multi-Camera Systems](https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html) - 360° field coverage
- [Outlier Rejection](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html) - Robust pose filtering

**📊 Competition Integration**
- [PathPlanner Integration](https://pathplanner.dev/pplib-getting-started.html) - Vision-corrected autonomous
- [Real-Time Localization](https://github.com/PhotonVision/photonvision) - Live pose tracking
- [Auto-Scoring Systems](https://docs.photonvision.org/en/latest/docs/examples/index.html) - Competition-ready alignment

**⚡ Performance Optimization**
- [FPGA Processing](https://www.ni.com/en-us/shop/labview/add-ons/third-party/ni-vision-for-labview.html) - Ultra-low latency detection
- [Edge Computing](https://developer.nvidia.com/embedded/jetson-nano-developer-kit) - High-performance coprocessors

---

**🚀 Ready to give your robot perfect vision?** Start with basic tag detection, master pose estimation, then integrate with professional vision systems for championship-level autonomous performance!