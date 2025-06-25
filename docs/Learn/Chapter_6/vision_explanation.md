# Vision Processing: Getting Started

## Overview

Vision processing turns your robot's camera into smart eyes that can recognize targets, track game pieces, and navigate autonomously. Instead of relying solely on sensors, your robot can "see" and react to the field like a human driver - but with perfect consistency.

**What you'll build:** A robot that can detect and track targets using computer vision for autonomous aiming and navigation.

**Next steps:** Advanced AprilTag tracking, neural networks, and real-time target recognition using [PhotonVision](https://docs.photonvision.org/) and [LimelightLib](https://docs.limelightvision.io/).

---

## 1. Why Vision Processing?

### The Power of Robot Vision
**Manual Targeting vs Vision:**
```java
// Manual: Driver aims manually
shooterSubsystem.setAngle(driverEstimatedAngle);

// Vision: Robot automatically finds and tracks target
double targetAngle = vision.getTargetAngle();
shooterSubsystem.setAngle(targetAngle);
```

**Real Benefits:**
- **Autonomous scoring** - Perfect aim every time
- **Driver assistance** - Help human pilots during teleop
- **Game piece detection** - Find balls, cones, cubes automatically
- **Navigation aids** - Use field landmarks for positioning
- **Consistent performance** - No human error in targeting

**FRC Applications:**
- Auto-aiming for shooters
- Autonomous ball collection
- Precise docking/charging
- Field-relative navigation
- Driver camera feeds

---

## 2. Vision Processing Pipeline

### The Step-by-Step Process

**1. Image Capture**
```
Camera → Raw Image Frame (640x480 pixels, 30 FPS)
```

**2. Preprocessing** 
```
Raw Image → Color Space Conversion → Filtered Image
```

**3. Target Detection**
```
Filtered Image → Contour Detection → Target Candidates
```

**4. Target Analysis**
```
Candidates → Filtering → Distance/Angle Calculations
```

**5. Robot Action**
```
Target Data → Robot Code → Motor Commands
```

### Basic Vision Workflow
```java
// Simplified pipeline
while (robot.isEnabled()) {
    // 1. Capture frame
    Mat frame = camera.getFrame();
    
    // 2. Filter for target color (e.g., green retroreflective tape)
    Mat filtered = filterForGreen(frame);
    
    // 3. Find contours (shapes)
    List<Contour> contours = findContours(filtered);
    
    // 4. Filter for target-shaped contours
    List<Target> targets = filterTargets(contours);
    
    // 5. Use best target for robot control
    if (!targets.isEmpty()) {
        Target best = getBestTarget(targets);
        double angle = calculateAngle(best);
        robotCode.setTargetAngle(angle);
    }
}
```

---

## 3. Vision Architecture Options

### Option 1: RoboRIO Vision (Simplest)
**Setup:** Camera → RoboRIO → Robot Code

```java
public class VisionSubsystem extends SubsystemBase {
    private final UsbCamera camera;
    private final CvSink cvSink;
    private final Mat frame = new Mat();
    
    public VisionSubsystem() {
        // Camera setup on roboRIO
        camera = CameraServer.startAutomaticCapture();
        camera.setResolution(320, 240);  // Lower resolution for performance
        camera.setFPS(15);               // Lower FPS for roboRIO
        
        cvSink = CameraServer.getVideo();
    }
    
    @Override
    public void periodic() {
        // Grab frame (non-blocking)
        if (cvSink.grabFrame(frame) == 0) {
            return; // No new frame
        }
        
        // Process frame
        processFrame(frame);
    }
    
    private void processFrame(Mat frame) {
        // Your vision pipeline here
        List<Target> targets = visionPipeline.process(frame);
        
        // Send results to robot
        if (!targets.isEmpty()) {
            SmartDashboard.putNumber("Target Angle", targets.get(0).angle);
        }
    }
}
```

**Pros:** Simple, everything in one place
**Cons:** Limited processing power, can slow robot code

### Option 2: Coprocessor Vision (Recommended)
**Setup:** Camera → Raspberry Pi/Jetson → NetworkTables → RoboRIO

```python
# Python vision code on Raspberry Pi
import cv2
import numpy as np
from networktables import NetworkTables

# Initialize NetworkTables
NetworkTables.initialize(server='10.TE.AM.2')  # Your robot IP
vision_table = NetworkTables.getTable('vision')

# Camera setup
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    ret, frame = camera.read()
    if not ret:
        continue
    
    # Vision pipeline
    targets = process_frame(frame)
    
    # Send to robot
    if targets:
        best_target = targets[0]
        vision_table.putNumber('target_angle', best_target.angle)
        vision_table.putNumber('target_distance', best_target.distance)
        vision_table.putBoolean('target_found', True)
    else:
        vision_table.putBoolean('target_found', False)
```

```java
// Robot code receives vision data
public class ShooterSubsystem extends SubsystemBase {
    private final NetworkTableEntry targetAngle;
    private final NetworkTableEntry targetFound;
    
    public ShooterSubsystem() {
        NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("vision");
        targetAngle = visionTable.getEntry("target_angle");
        targetFound = visionTable.getEntry("target_found");
    }
    
    public boolean hasTarget() {
        return targetFound.getBoolean(false);
    }
    
    public double getTargetAngle() {
        return targetAngle.getDouble(0.0);
    }
}
```

**Pros:** Fast processing, doesn't slow robot code
**Cons:** More complex setup, additional hardware

### Option 3: Driver Station Vision
**Setup:** Camera → RoboRIO → Driver Station → NetworkTables → Robot

**Pros:** Powerful laptop processing
**Cons:** High latency, bandwidth limitations

---

## 4. Basic Vision Pipeline Implementation

### Step 1: Color Filtering
```java
public class VisionPipeline {
    // HSV ranges for green retroreflective tape
    private final Scalar lowerGreen = new Scalar(40, 50, 50);
    private final Scalar upperGreen = new Scalar(80, 255, 255);
    
    public Mat filterForTarget(Mat input) {
        Mat hsv = new Mat();
        Mat mask = new Mat();
        
        // Convert to HSV color space
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
        
        // Filter for green color
        Core.inRange(hsv, lowerGreen, upperGreen, mask);
        
        // Clean up noise
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);
        
        return mask;
    }
}
```

### Step 2: Contour Detection
```java
public List<MatOfPoint> findContours(Mat mask) {
    List<MatOfPoint> contours = new ArrayList<>();
    Mat hierarchy = new Mat();
    
    // Find contours
    Imgproc.findContours(mask, contours, hierarchy, 
                        Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
    
    return contours;
}
```

### Step 3: Target Filtering
```java
public class Target {
    public double centerX;
    public double centerY;
    public double area;
    public double angle;
    public double distance;
}

public List<Target> filterTargets(List<MatOfPoint> contours) {
    List<Target> targets = new ArrayList<>();
    
    for (MatOfPoint contour : contours) {
        double area = Imgproc.contourArea(contour);
        
        // Filter by size
        if (area < 100 || area > 10000) {
            continue;
        }
        
        // Get bounding rectangle
        Rect boundingRect = Imgproc.boundingRect(contour);
        double aspectRatio = (double) boundingRect.width / boundingRect.height;
        
        // Filter by aspect ratio (targets are typically wider than tall)
        if (aspectRatio < 1.2 || aspectRatio > 3.0) {
            continue;
        }
        
        // Create target object
        Target target = new Target();
        target.centerX = boundingRect.x + boundingRect.width / 2.0;
        target.centerY = boundingRect.y + boundingRect.height / 2.0;
        target.area = area;
        
        targets.add(target);
    }
    
    // Sort by area (biggest first)
    targets.sort((a, b) -> Double.compare(b.area, a.area));
    
    return targets;
}
```

### Step 4: Angle and Distance Calculation
```java
public void calculateTargetInfo(Target target, int imageWidth, int imageHeight) {
    // Camera parameters (calibrate these for your camera)
    double focalLength = 300;  // pixels
    double targetWidth = 14.5; // inches (actual target width)
    
    // Calculate horizontal angle
    double centerX = imageWidth / 2.0;
    double pixelError = target.centerX - centerX;
    target.angle = Math.atan(pixelError / focalLength);
    
    // Estimate distance (rough approximation)
    double targetPixelWidth = Math.sqrt(target.area * 2); // Approximate width
    target.distance = (targetWidth * focalLength) / targetPixelWidth;
}
```

---

## 5. Integration with Robot Code

### Auto-Aiming Shooter
```java
public class AutoAimCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final VisionSubsystem vision;
    private final PIDController turnController;
    
    public AutoAimCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
        this.shooter = shooter;
        this.vision = vision;
        this.turnController = new PIDController(0.1, 0.0, 0.01);
        
        addRequirements(shooter);
    }
    
    @Override
    public void execute() {
        if (vision.hasTarget()) {
            double targetAngle = vision.getTargetAngle();
            double output = turnController.calculate(0, targetAngle); // Want angle error to be 0
            shooter.setTurretSpeed(output);
        } else {
            shooter.setTurretSpeed(0);
        }
    }
    
    @Override
    public boolean isFinished() {
        return vision.hasTarget() && Math.abs(vision.getTargetAngle()) < 0.05; // Within tolerance
    }
}
```

### Vision-Assisted Driving
```java
public class DriveWithVisionAssist extends CommandBase {
    private final SwerveDrive drive;
    private final VisionSubsystem vision;
    private final XboxController controller;
    
    @Override
    public void execute() {
        // Get driver input
        double xSpeed = -controller.getLeftY();
        double ySpeed = -controller.getLeftX();
        double rotation = -controller.getRightX();
        
        // Add vision assist for rotation
        if (controller.getAButton() && vision.hasTarget()) {
            double visionCorrection = vision.getTargetAngle() * 0.1; // Gentle assist
            rotation += visionCorrection;
        }
        
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotation);
        drive.drive(speeds);
    }
}
```

---

## 6. Real-World Examples

### AprilTag Detection (Simple)
```java
// Using WPILib's AprilTag detector
AprilTagDetector detector = new AprilTagDetector();
detector.addFamily(AprilTagFamily.TAG_16h5);

public void processAprilTags(Mat frame) {
    AprilTagDetection[] detections = detector.detect(frame);
    
    for (AprilTagDetection detection : detections) {
        int tagID = detection.getId();
        Pose3d tagPose = detection.getPose();
        
        SmartDashboard.putNumber("Tag " + tagID + " X", tagPose.getX());
        SmartDashboard.putNumber("Tag " + tagID + " Y", tagPose.getY());
    }
}
```

### Game Piece Detection
```python
# Detect orange balls/cones
def detect_game_pieces(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Orange color range
    lower_orange = np.array([10, 100, 100])
    upper_orange = np.array([20, 255, 255])
    
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    game_pieces = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 500:  # Filter small detections
            x, y, w, h = cv2.boundingRect(contour)
            center_x = x + w // 2
            center_y = y + h // 2
            game_pieces.append({'x': center_x, 'y': center_y, 'area': area})
    
    return game_pieces
```

---

## 7. Performance Optimization

### Camera Settings
```java
// Optimize for performance
camera.setResolution(320, 240);  // Lower resolution = faster processing
camera.setFPS(15);               // 15 FPS often sufficient
camera.setExposure(0);           // Auto exposure off for consistent lighting
camera.setBrightness(30);        // Adjust for your field conditions
```

### Processing Optimizations
```java
// Process every other frame for performance
private int frameCount = 0;

@Override
public void periodic() {
    frameCount++;
    if (frameCount % 2 == 0) {  // Process every other frame
        processFrame();
    }
}

// Region of Interest (ROI) processing
public Mat getROI(Mat frame) {
    // Only process center portion where targets likely appear
    int x = frame.width() / 4;
    int y = frame.height() / 4;
    int width = frame.width() / 2;
    int height = frame.height() / 2;
    
    Rect roi = new Rect(x, y, width, height);
    return new Mat(frame, roi);
}
```

---

## 8. Debugging and Tuning

### Essential Telemetry
```java
@Override
public void periodic() {
    // Camera info
    SmartDashboard.putNumber("Camera FPS", camera.getActualFPS());
    SmartDashboard.putBoolean("Camera Connected", camera.isConnected());
    
    // Vision results
    SmartDashboard.putBoolean("Target Found", hasTarget());
    SmartDashboard.putNumber("Target Count", targets.size());
    
    if (hasTarget()) {
        Target best = getBestTarget();
        SmartDashboard.putNumber("Target Angle", best.angle);
        SmartDashboard.putNumber("Target Distance", best.distance);
        SmartDashboard.putNumber("Target Area", best.area);
    }
    
    // Performance monitoring
    SmartDashboard.putNumber("Processing Time", processingTimeMs);
}
```

### Common Issues & Solutions

**Problem:** No targets detected
- **Solution:** Check color ranges, lighting conditions, camera exposure

**Problem:** False positive detections
- **Solution:** Tighten filtering criteria (area, aspect ratio, etc.)

**Problem:** Intermittent target loss
- **Solution:** Add target tracking/prediction, improve filtering

**Problem:** Poor performance/lag
- **Solution:** Lower resolution/FPS, optimize pipeline, use coprocessor

**Problem:** Inaccurate distance measurements
- **Solution:** Calibrate camera, use stereo vision, add known target size

---

## 9. Practice Project

**Build this step-by-step:**

1. **Camera Setup** - Get camera streaming to dashboard
2. **Basic Detection** - Detect simple colored objects
3. **Target Filtering** - Filter by size and shape
4. **Angle Calculation** - Calculate target angle from center
5. **Robot Integration** - Use vision data to control robot
6. **Auto-Aiming** - Create automatic targeting system

**Success Criteria:**
- Consistent target detection under field lighting
- Accurate angle calculations within 2-3 degrees
- Smooth auto-aiming without oscillation
- Good performance (>10 FPS processing)

---

## Where to Go Next

**Ready for professional vision? Explore these:**

**🎯 Pre-Built Solutions**
- [PhotonVision](https://docs.photonvision.org/) - Complete vision processing suite
- [LimelightLib](https://docs.limelightvision.io/) - Plug-and-play vision computer
- [Chameleon Vision](https://chameleon-vision.readthedocs.io/) - Open-source vision processing

**🔧 Advanced Techniques**
- [Camera Calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) - Accurate 3D positioning
- [Pose Estimation](https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html) - 6DOF target tracking
- [Machine Learning](https://www.tensorflow.org/) - Neural network object detection

**📊 Multi-Camera Systems**
- [Stereo Vision](https://docs.opencv.org/4.x/dd/d53/tutorial_py_depthmap.html) - Depth perception
- [360° Coverage](https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html) - Multiple camera coordination

**⚡ Real-Time Performance**
- [GPU Acceleration](https://opencv.org/platforms/cuda/) - CUDA-accelerated processing
- [FPGA Vision](https://www.ni.com/en-us/shop/labview/add-ons/third-party/ni-vision-for-labview.html) - Ultra-low latency processing

---

**🚀 Ready to give your robot eyes?** Start with basic color detection, master the fundamentals, then explore professional vision solutions for competition-winning autonomous performance!