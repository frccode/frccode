# AprilTag Processing: Getting Started

## Overview

AprilTags are like QR codes for robots - fiducial markers that provide precise position and orientation data. Unlike basic vision targets, AprilTags tell your robot exactly where it is on the field and can enable centimeter-accurate autonomous navigation and positioning.

A **fiducial marker** is a specially designed visual pattern placed in the environment to serve as a reference point for computer vision systems.

**What you'll build:** A robot that uses AprilTags for precise field localization and automated alignment.



---

## 1. What Are AprilTags?
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
- **6 item Pose** - X, Y, Z position + roll, pitch, yaw rotation
- **Robust Detection** - Works with partial occlusion and poor lighting
- **No Calibration Required** - For basic centering applications

**Tag Features:**
- Multiple tags placed at known positions throughout the field
- Each tag is 6.5" × 6.5" with unique ID
- Tags mounted flat on field elements
- Standard 36h11 family used

**Apllications:**
AprilTags enable a range of advanced capabilities for robotics, including auto-alignment by driving until a tag is centered, precise pose estimation to determine the robot's exact position, autonomous navigation using field landmarks, and serving as a backup to odometry by correcting for wheel or gyro drift.

> **Learn More:**  
> [PhotonVision: About AprilTags](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/about-apriltags.html) – In-depth explanation of AprilTag technology, detection principles, and practical usage in robotics.

---
## 2. How AprilTag Detection Works

AprilTag detection is a robust computer vision process that enables robots to identify and localize fiducial markers on the field. 

While alot of this process has been abstracted by 3rd party Vision solution vendors such as Limelight and PhotonVision, understanding the detection pipeline helps teams tune their systems for speed and accuracy, and troubleshoot issues. Below is a simplified diagram explaining apriltag processing:


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

For a deeper dive on this topic, check out [WPILib: What Are AprilTags?](https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html#what-are-apriltags) for a comprehensive overview of AprilTag technology, detection algorithms, and their application in FRC robotics.


---

## 3. Basic Implementation

Below is a simple get started project on setting up a vision system using a USB Camera and a roborio. Each following step from 3-8 will guide you through setting up all the code needed to run in.
> **Reference:**  
> [Using the CameraServer on the RoboRIO (WPILib)](https://docs.wpilib.org/en/2021/docs/software/vision-processing/roborio/using-the-cameraserver-on-the-roborio.html) – Guide to setting up USB cameras and streaming video for vision processing on FRC robots.

### Simple Tag Detection
```java

public class VisionSubsystem extends SubsystemBase {
    private UsbCamera camera;
    private final AprilTagDetector detector;
    private final CvSink cvSink;
    private final Mat frame = new Mat();

    private AprilTagDetection[] lastDetections = new AprilTagDetection[0];

    public VisionSubsystem() {
        // Camera setup (startup thread for grayscale conversion)
        new Thread(() -> {
            camera = CameraServer.startAutomaticCapture();
            camera.setResolution(640, 480);

            CvSink cvSinkThread = CameraServer.getVideo();
            CvSource outputStream = CameraServer.putVideo("Gray", 640, 480);

            Mat source = new Mat();
            Mat output = new Mat();

            while (!Thread.interrupted()) {
                if (cvSinkThread.grabFrame(source) == 0) {
                    continue;
                }
                // Convert to grayscale for AprilTag detection
                Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
                outputStream.putFrame(output);
            }
        }).start();

        // AprilTag detector setup
        detector = new AprilTagDetector();
        detector.addFamily("TAG_36h11"); // FRC standard

        // Configure detection parameters
        AprilTagDetector.Config config = detector.getConfig();
        config.quadDecimate = 2;  // Downsample by factor of 2
        config.numThreads = 2;        // Use 2 CPU cores
        detector.setConfig(config);

        // Get CvSink for detection (main thread)
        cvSink = CameraServer.getVideo();
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

### Auto-Alignment (Simple)

This code provdes a simple align to the nearest apriltag using a rotation PID controller. See [PID Controllers Explained](../PID.md) for explaination on PID controllers

```java
public class AlignToTagCommand extends Command {
    private final SwerveDrive drive;
    private final VisionSubsystem vision;
    private final int targetTagID;
    private final PIDController rotationController;
    
    public AlignToTagCommand(SwerveDrive drive, VisionSubsystem vision, int tagID) {
        this.drive = drive;
        this.vision = vision;
        this.targetTagID = tagID;
        this.rotationController = new PIDController(0.1, 0.0, 0.01); // Should be tuned based off user preference
        
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

## 5. Performance Optimization

These are some settings you can change to optimize performance for your roborio usb camera setup.

### Detection Parameters
```java
public void configureForPerformance() {
    AprilTagDetector.Config config = detector.getConfig();
    
    // Fast detection (lower accuracy)
    config.quadDecimate = 4;    // Downsample more aggressively
    config.sigma = 0.8;             // More blur (faster but less precise)
    config.numThreads = 4;          // Use more CPU cores for faster processing
    config.refineEdges = false;     // Skip sub-pixel refinement
    
    detector.setConfig(config);
}

public void configureForAccuracy() {
    AprilTagDetector.Config config = detector.getConfig();
    
    // Accurate detection (slower)
    config.quadDecimate = 1;    // No downsampling
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

## 6. Debugging and Troubleshooting

Below is an example logging setup used to debug a vision system. These can be copy pasted into your vision subsystem periodic.

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


## 7. Practice Project

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

### Next Steps
Continue with [Pose Estimators](3_pose_estimators_and_kalman_filters_and_CS.md) to learn how to fuse vision and sensor data for even more accurate robot localization and autonomous control.