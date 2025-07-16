# Vision Processing: Getting Started

## Overview

Vision processing turns your robot's camera into smart eyes that can recognize targets, track game pieces, and navigate autonomously. Instead of relying solely on sensors, your robot can "see" and react to the field like a human driver.

Since the introduction of vision assisted programming into FRC, there has been a spike in robot capabilities and the speed at which they can perform tasks. By givng your robot eyes and your code an extra data input, it played a key role in drammatically changing uping the playing field programmers were on when it comes to fast, competative, robotics. 

> **Note:** Much of this documentation/examples is based off [PhotonVision](https://docs.photonvision.org/en/latest/docs/description.html) and [WPILib Vision Processing](https://docs.wpilib.org/en/stable/docs/software/vision-processing/index.html) documentation that provides a more in depth explanation on the topics cover in this section. This section is meant to get users started on a simple vision program for those without any previous knowledge. However, it is recommended to check out these documentations for a more in depth understanding of vision systems.



**What you'll build:** 
A robot that can detect and track targets using computer vision for autonomous aiming and navigation.


## 1. Why Vision Processing?

### The Power of Robot Vision
**Manual Targeting vs Vision:**

Here's a simple code example of how vision can automate driver control for faster, more reliable scoring/loading tasks.

```java
// Manual: Driver aims manually
shooterSubsystem.setAngle(xboxAux.getLeftY());

// Vision: Robot automatically finds and tracks target
double targetAngle = vision.getTargetAngle();
shooterSubsystem.setAngle(targetAngle);
```

**Example Benefits:**
- **Autonomous scoring** - Perfect aim every time
- **Driver assistance** - Help human pilots during teleop
- **Game piece detection** - Find balls, cones, cubes automatically
- **Navigation aids** - Use field landmarks for positioning
- **Consistent performance** - No human error in targeting

**Example Applications:**
- Auto-aiming for shooters 
- Autonomous ball collection
- Precise docking/charging
- Field-relative navigation
- Driver camera feeds

---

## 2. Vision Processing Pipeline

This is a oversimplified model of how general vision(apriltags, relflective tape, objects) are process and turned into data that programmers can use.

A **vision pipeline** is a sequence of image processing steps that transforms raw camera input into useful data for your robot code. Each pipeline is designed to detect specific features—such as colors, shapes, or fiducial markers (like AprilTags)—and extract measurements like position, angle, or distance.
[[PhotonVision documentation on pipelines](https://docs.photonvision.org/en/latest/docs/pipelines/about-pipelines.html#what-is-a-pipeline)].

### The Step-by-Step Process

**1. Image Capture**: The image is captured through a usb camera. Team's have varying preferences for framerates, resolution, exposure, etc. when it comes to selecting their camera based on what it's needed for. 
>**Note:** As a simple example of differing specs for differing purposes, in apriltag processing, teams prefer to look at black and white cameras to process the black and white apirltags while in object detection, teams look for color cameras that can detect color objects. 

```
Camera → Raw Image Frame (640x480 pixels, 30 FPS)
```


**2. Preprocessing**: The image is then taken through a series of steps that "color grade" the image so the internal vision pipline can identify known artifiacts.
```
Raw Image → Color Space Conversion → Filtered Image
```

**3. Target Detection**: The vision artifacts are identified by the vision pipline and translated into variables used in code.
```
Filtered Image → Contour Detection → Target Candidates
```
>**Note:** Preprocessing and target detection are often performed on a coprocessor such as an Orange Pi or Raspberry Pi. These devices are used to offload computationally intensive tasks from the main system, enabling real-time image analysis and efficient resource utilization in embedded vision applications.

**4. Target Analysis, Calculation, and Robot Action** This is the point where the user takes the resulting data and applies it to their robot logic.
```
Candidates → Filtering → Distance/Angle Calculations
Target Data → Robot Code → Motor Commands
```

---

## 3. Basic Vision Pipeline Implementation

Below is a simplified Java vision pipeline using OpenCV, similar to what many FRC teams use for retroreflective tape detection. This example covers color filtering, contour detection, and target filtering.

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

---

For a full working example and more advanced features, see the [PhotonVision FRC vision processing repository](https://github.com/PhotonVision/photonvision).


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

## 5. Performance Optimization

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

## 6. Debugging and Tuning

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

These are common problems with suggested solutions. However, these are not "one size fits all"

**Problem:** No targets detected
- **Solution:** Check color ranges, lighting conditions, camera exposure

**Problem:** False positive detections
- **Solution:** Tighten filtering criteria (area, aspect ratio, etc.)

**Problem:** Intermittent target loss
- **Solution:** Add target tracking/prediction, improve filtering

**Problem:** Poor performance/lag
- **Solution:** Lower resolution/FPS, use coprocessor

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

For a deeper dive into AprilTag detection and usage, see the [AprilTag Processing Getting Started](https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/index.html).



---

