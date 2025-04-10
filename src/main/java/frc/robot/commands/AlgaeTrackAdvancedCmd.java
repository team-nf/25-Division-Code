// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/* Advanced version of AlgaeTrackCmd with object tracking and velocity estimation */
public class AlgaeTrackAdvancedCmd extends Command {
  // Selection algorithm enum
  public enum SelectionAlgorithm {
    LOWEST_FIRST,        // Select the lowest algae in frame (highest ty value)
    LOWER_CENTER_CLOSEST, // Select algae closest to bottom center
    BIGGEST,             // Select largest algae by area
    FASTEST_MOVING,      // Select algae with highest velocity
    APPROACHING          // Select algae that is moving toward robot
  }
  
  // Tracked object class to store object state between frames
  private class TrackedObject {
    public int id;                       // Unique identifier
    public RawDetection detection;       // Current detection data
    public Translation2d position;       // Current position (tx, ty)
    public Translation2d velocity;       // Estimated velocity (pixels/second or degrees/second)
    public double area;                  // Current area
    public double distance;              // Estimated distance in meters
    public double lastUpdateTime;        // Time of last update
    public int frameCount;               // Number of frames this object has been tracked
    public boolean isMatched;            // Whether this object was matched in current frame
    
    // Store position history for filtering
    private ArrayList<Translation2d> positionHistory = new ArrayList<>();
    private static final int MAX_HISTORY = 5;
    
    // Create a new tracked object
    public TrackedObject(int id, RawDetection detection, double currentTime) {
      this.id = id;
      this.detection = detection;
      this.position = new Translation2d(detection.txnc, detection.tync);
      this.velocity = new Translation2d(0, 0);
      this.area = detection.ta;
      this.distance = estimateDistance(detection);
      this.lastUpdateTime = currentTime;
      this.frameCount = 1;
      this.isMatched = true;
      
      // Initialize position history
      positionHistory.add(this.position);
    }
    
    // Update tracked object with new detection
    public void update(RawDetection detection, double currentTime) {
      // Store previous values for velocity calculation
      Translation2d prevPosition = this.position;
      double prevTime = this.lastUpdateTime;
      
      // Update with new values
      this.detection = detection;
      this.position = new Translation2d(detection.txnc, detection.tync);
      this.area = detection.ta;
      this.lastUpdateTime = currentTime;
      this.frameCount++;
      this.isMatched = true;
      
      // Add to position history
      positionHistory.add(this.position);
      if (positionHistory.size() > MAX_HISTORY) {
        positionHistory.remove(0);
      }
      
      // Calculate velocity (change in position / change in time)
      double dt = currentTime - prevTime;
      if (dt > 0) {
        double vx = (this.position.getX() - prevPosition.getX()) / dt;
        double vy = (this.position.getY() - prevPosition.getY()) / dt;
        
        // Apply exponential smoothing to velocity
        double alpha = 0.7; // Smoothing factor
        double smoothedVx = alpha * vx + (1 - alpha) * this.velocity.getX();
        double smoothedVy = alpha * vy + (1 - alpha) * this.velocity.getY();
        
        this.velocity = new Translation2d(smoothedVx, smoothedVy);
      }
      
      // Update distance estimate
      this.distance = estimateDistance(detection);
    }
    
    // Predict where this object will be in the future
    public Translation2d predictPosition(double futureTime) {
      double dt = futureTime - this.lastUpdateTime;
      return new Translation2d(
        this.position.getX() + this.velocity.getX() * dt,
        this.position.getY() + this.velocity.getY() * dt
      );
    }
    
    // Get filtered position using a simple moving average
    public Translation2d getFilteredPosition() {
      double sumX = 0;
      double sumY = 0;
      
      for (Translation2d pos : positionHistory) {
        sumX += pos.getX();
        sumY += pos.getY();
      }
      
      return new Translation2d(
        sumX / positionHistory.size(),
        sumY / positionHistory.size()
      );
    }
    
    // Calculate score for matching with a new detection
    public double calculateMatchScore(RawDetection detection, double currentTime) {
      // Position error - where we expect the object to be vs where the detection is
      Translation2d predictedPos = predictPosition(currentTime);
      Translation2d detectionPos = new Translation2d(detection.txnc, detection.tync);
      
      // Calculate Euclidean distance between predicted position and detection
      double positionError = predictedPos.getDistance(detectionPos);
      
      // Calculate area similarity (ratio of areas, capped at 1.0)
      double areaRatio = Math.min(detection.ta / this.area, this.area / detection.ta);
      
      // Calculate time factor - objects tracked for longer are more reliable
      double timeFactor = Math.min(1.0, this.frameCount / 10.0);
      
      // Calculate position error adjusted by object velocity
      // Fast-moving objects can have larger position errors
      double velocityMagnitude = Math.hypot(this.velocity.getX(), this.velocity.getY());
      double adjustedPositionError = positionError / (1.0 + velocityMagnitude * 0.1);
      
      // Calculate final score (lower is better)
      return adjustedPositionError * (2.0 - areaRatio) * (2.0 - timeFactor);
    }
  }
  
  // Subsystem and limelight configuration
  private CommandSwerveDrivetrain m_swerve;
  private final String limelightName = "limelight-obj";
  
  // Algorithm selection and filtering flags
  private static SelectionAlgorithm currentAlgorithm = SelectionAlgorithm.LOWEST_FIRST;
  private boolean enableRectangularCheck = true;
  private boolean enableAreaRatioCheck = true;
  private boolean usePIDController = false;
  
  // Target point in bounding box using Cartesian coordinates (-1 to 1, -1 to 1)
  // where (0,0) is center, (-1,0) is left center, (1,0) is right center
  // (0,1) is top center, (0,-1) is bottom center
  private static double targetOffsetX = -0.8;  // Default: 80% toward left edge from center
  private static double targetOffsetY = 0.0;   // Default: vertical center
  
  // Tracking configuration
  private Map<Integer, TrackedObject> trackedObjects = new HashMap<>();
  private int nextObjectId = 0;
  private TrackedObject selectedObject = null;
  private double lastProcessTime = 0;
  private static final double MAX_MATCH_SCORE = 30.0;  // Maximum score for matching objects
  private static final double MAX_TRACK_AGE = 1.0;     // Time in seconds before dropping tracking
  
  // Constants
  private static final double TX_MIN = -29.8;
  private static final double TX_MAX = 29.8;
  private static final double TY_MIN = -24.85;
  private static final double TY_MAX = 24.85;
  private static final double kForwardP = 3.0;
  private static final double kRotationP = 1.8;
  private static final double MAX_ASPECT_RATIO = 1.3;
  private static final double MIN_AREA_RATIO = 0.65;
  private static final double EDGE_MARGIN = 5.0;
  
  // Constants for distance estimation
  private static final double ALGAE_DIAMETER_METERS = 0.24;  // Approximate algae ball diameter
  private static final double CAMERA_HEIGHT_METERS = 0.65;   // Height of camera from ground
  private static final double CAMERA_PITCH_RADIANS = Math.toRadians(25); // Camera angle from horizontal
  
  // Filters for smoothing
  private LinearFilter velocityFilter = LinearFilter.movingAverage(5);
  private LinearFilter rotationFilter = LinearFilter.movingAverage(3);

  // PID controllers
  private final PIDController rotationPID;
  private final PIDController forwardPID;
  
  // Drive configuration
  private final SwerveRequest.RobotCentric roboDrive;

  public AlgaeTrackAdvancedCmd(CommandSwerveDrivetrain swerve) {
    m_swerve = swerve;
    addRequirements(m_swerve);
    
    // Create PID controllers
    rotationPID = new PIDController(kRotationP, 0, 0);
    forwardPID = new PIDController(kForwardP, 0, 0);
    
    // Set input range for rotation PID (helps with wrap-around)
    rotationPID.enableContinuousInput(-1.0, 1.0);
    
    double maxSpeed = MetersPerSecond.of(5).in(MetersPerSecond);
    roboDrive = new SwerveRequest.RobotCentric()
      .withDeadband(maxSpeed * 0.035)
      .withDriveRequestType(DriveRequestType.Velocity);
  }

  @Override
  public void initialize() {
    // Set limelight pipeline for neural detection and reset tracking
    LimelightHelpers.setPipelineIndex(limelightName, 0);
    
    // Reset tracking state
    trackedObjects.clear();
    selectedObject = null;
    nextObjectId = 0;
    lastProcessTime = getCurrentTime();
    
    // Reset PID controllers
    rotationPID.reset();
    forwardPID.reset();
  }

  @Override
  public void execute() {
    // Get current time for tracking
    double currentTime = getCurrentTime();
    double deltaTime = currentTime - lastProcessTime;
    lastProcessTime = currentTime;
    
    // Process detections, update tracking, and drive toward target if found
    processLimelightDetections(currentTime);
    
    if (selectedObject != null) {
      // Use the tracked object's filtered position for smoother tracking
      RawDetection selectedDetection = selectedObject.detection;
      
      // Calculate drive values
      double targetPointX = calculateTargetPoint(selectedDetection);
      
      double forwardVelocity;
      double rotationVelocity;
      
      if (usePIDController) {
        // Use PID controllers
        double mappedTX = map(targetPointX, TX_MIN, TX_MAX, -1.0, 1.0);
        double mappedTY = map(selectedDetection.tync, TY_MIN, TY_MAX, 0.0, 1.0);
        
        rotationVelocity = -rotationPID.calculate(mappedTX, 0); // Target is center (0)
        forwardVelocity = -forwardPID.calculate(mappedTY, 0);   // Target is 0 (minimum distance)
      } else {
        // Use simple P controller with velocity prediction
        double[] controlValues = calculateControlValuesWithPrediction(selectedObject, targetPointX, deltaTime);
        forwardVelocity = controlValues[0];
        rotationVelocity = controlValues[1];
      }
      
      // Apply filtering for smoother motion
      forwardVelocity = velocityFilter.calculate(forwardVelocity);
      rotationVelocity = rotationFilter.calculate(rotationVelocity);
      
      // Log control values
      SmartDashboard.putNumber("AlgaeAdv_ForwardVelocity", forwardVelocity);
      SmartDashboard.putNumber("AlgaeAdv_RotationVelocity", rotationVelocity);
      SmartDashboard.putBoolean("AlgaeAdv_UsingPID", usePIDController);
      
      // Drive robot using calculated velocities
      m_swerve.setControl(
        roboDrive
          .withVelocityX(-forwardVelocity) // Forward velocity
          .withVelocityY(0.0)               // No side-to-side motion
          .withRotationalRate(rotationVelocity) // Rotation velocity
      );
    } else {
      // No valid target, stop robot
      m_swerve.setControl(
        roboDrive
          .withVelocityX(0)
          .withVelocityY(0)
          .withRotationalRate(0)
      );
    }
  }

  private void processLimelightDetections(double currentTime) {
    boolean hasTarget = LimelightHelpers.getTV(limelightName);
    SmartDashboard.putBoolean("LLAdv_HasTarget", hasTarget);
    
    if (hasTarget) {
      // Mark all existing tracked objects as unmatched
      for (TrackedObject obj : trackedObjects.values()) {
        obj.isMatched = false;
      }
      
      updateLimelightData();
      RawDetection[] detections = LimelightHelpers.getRawDetections(limelightName);
      
      if (detections != null && detections.length > 0) {
        SmartDashboard.putNumber("LLAdv_DetectionCount", detections.length);
        
        // Track all valid detections
        trackDetections(detections, currentTime);
        
        // Clean up old or unmatched tracked objects
        cleanupTrackedObjects(currentTime);
        
        // Select object based on algorithm
        selectTrackingTarget();
      } else {
        SmartDashboard.putNumber("LLAdv_DetectionCount", 0);
        selectedObject = null;
      }
    } else {
      SmartDashboard.putNumber("LLAdv_DetectionCount", 0);
      selectedObject = null;
    }
    
    displayTrackingData();
  }
  
  private void trackDetections(RawDetection[] detections, double currentTime) {
    // Process each detection and match with existing objects or create new ones
    for (RawDetection detection : detections) {
      // Skip invalid detections
      if (!isBoxSquarish(detection) || !hasValidAreaRatio(detection)) {
        continue;
      }
      
      boolean matched = false;
      double bestScore = MAX_MATCH_SCORE;
      TrackedObject bestMatch = null;
      
      // Try to match with existing tracked objects
      for (TrackedObject obj : trackedObjects.values()) {
        if (obj.isMatched) {
          continue; // Already matched to another detection
        }
        
        double score = obj.calculateMatchScore(detection, currentTime);
        if (score < bestScore) {
          bestScore = score;
          bestMatch = obj;
        }
      }
      
      if (bestMatch != null) {
        // Update matched object
        bestMatch.update(detection, currentTime);
        matched = true;
      }
      
      if (!matched) {
        // Create new tracked object
        TrackedObject newObject = new TrackedObject(nextObjectId++, detection, currentTime);
        trackedObjects.put(newObject.id, newObject);
      }
    }
  }
  
  private void cleanupTrackedObjects(double currentTime) {
    // Remove objects that haven't been matched for too long
    trackedObjects.entrySet().removeIf(entry -> {
      TrackedObject obj = entry.getValue();
      double age = currentTime - obj.lastUpdateTime;
      return !obj.isMatched && age > MAX_TRACK_AGE;
    });
  }
  
  private void selectTrackingTarget() {
    selectedObject = null;
    
    if (trackedObjects.isEmpty()) {
      return;
    }
    
    // Select object based on algorithm
    switch (currentAlgorithm) {
      case LOWEST_FIRST:
        selectLowestObject();
        break;
      case LOWER_CENTER_CLOSEST:
        selectClosestToBottomCenter();
        break;
      case BIGGEST:
        selectBiggestObject();
        break;
      case FASTEST_MOVING:
        selectFastestMovingObject();
        break;
      case APPROACHING:
        selectApproachingObject();
        break;
      default:
        selectLowestObject();
        break;
    }
  }
  
  private void selectLowestObject() {
    // Select the lowest object in frame (highest ty value)
    double highestTY = -Double.MAX_VALUE;
    
    for (TrackedObject obj : trackedObjects.values()) {
      if (obj.position.getY() > highestTY) {
        highestTY = obj.position.getY();
        selectedObject = obj;
      }
    }
  }
  
  private void selectClosestToBottomCenter() {
    // Select object closest to bottom center
    double closestDistance = Double.MAX_VALUE;
    
    for (TrackedObject obj : trackedObjects.values()) {
      // Calculate distance to bottom center (approximately (0, TY_MIN))
      double distToBottomCenter = Math.sqrt(
          Math.pow(obj.position.getX(), 2) + 
          Math.pow(obj.position.getY() - TY_MIN, 2));
      
      if (distToBottomCenter < closestDistance) {
        closestDistance = distToBottomCenter;
        selectedObject = obj;
      }
    }
  }
  
  private void selectBiggestObject() {
    // Select the largest object by area
    double largestArea = -Double.MAX_VALUE;
    
    for (TrackedObject obj : trackedObjects.values()) {
      if (obj.area > largestArea) {
        largestArea = obj.area;
        selectedObject = obj;
      }
    }
  }
  
  private void selectFastestMovingObject() {
    // Select object with highest velocity
    double highestVelocity = -Double.MAX_VALUE;
    
    for (TrackedObject obj : trackedObjects.values()) {
      double velocity = Math.hypot(obj.velocity.getX(), obj.velocity.getY());
      
      if (velocity > highestVelocity && obj.frameCount > 3) {
        highestVelocity = velocity;
        selectedObject = obj;
      }
    }
    
    // Fall back to biggest if no objects have significant velocity
    if (selectedObject == null) {
      selectBiggestObject();
    }
  }
  
  private void selectApproachingObject() {
    // Select object that is moving toward the robot (negative Y velocity)
    double highestApproachRate = -Double.MAX_VALUE;
    
    for (TrackedObject obj : trackedObjects.values()) {
      // Negative Y velocity means object is moving down in the image (approaching)
      double approachRate = -obj.velocity.getY();
      
      if (approachRate > highestApproachRate && obj.frameCount > 3) {
        highestApproachRate = approachRate;
        selectedObject = obj;
      }
    }
    
    // Fall back to lowest if no objects are approaching
    if (selectedObject == null || highestApproachRate <= 0) {
      selectLowestObject();
    }
  }
  
  private void updateLimelightData() {
    // Update SmartDashboard with basic Limelight data
    SmartDashboard.putNumber("LLAdv_TX", LimelightHelpers.getTX(limelightName));
    SmartDashboard.putNumber("LLAdv_TY", LimelightHelpers.getTY(limelightName));
    SmartDashboard.putNumber("LLAdv_TA", LimelightHelpers.getTA(limelightName));
    SmartDashboard.putString("LLAdv_DetectorClass", LimelightHelpers.getDetectorClass(limelightName));
    
    // Display algorithm and filter settings
    SmartDashboard.putString("AlgaeAdv_Algorithm", currentAlgorithm.toString());
    SmartDashboard.putBoolean("AlgaeAdv_EnableRectCheck", enableRectangularCheck);
    SmartDashboard.putBoolean("AlgaeAdv_EnableAreaCheck", enableAreaRatioCheck);
    SmartDashboard.putNumber("AlgaeAdv_TrackedObjectCount", trackedObjects.size());
    SmartDashboard.putNumber("AlgaeAdv_TargetOffsetX", targetOffsetX);
    SmartDashboard.putNumber("AlgaeAdv_TargetOffsetY", targetOffsetY);
  }
  
  private double calculateTargetPoint(RawDetection detection) {
    // Calculate bounding box dimensions and center
    double[] dimensions = getBoundingBoxDimensions(detection);
    double width = dimensions[0];
    double height = dimensions[1];
    double minX = dimensions[2];
    double maxX = dimensions[3];
    double minY = dimensions[4];
    double maxY = dimensions[5];
    
    double centerX = (minX + maxX) / 2.0;
    double centerY = (minY + maxY) / 2.0;
    
    // Calculate target coordinates using the offset from center
    // Map from (-1,1) to actual pixel coordinates
    double targetX = centerX + (targetOffsetX * width / 2.0);
    double targetY = centerY + (targetOffsetY * height / 2.0);
    
    // Ensure target point is within bounds
    targetX = Math.max(minX, Math.min(maxX, targetX));
    targetY = Math.max(minY, Math.min(maxY, targetY));
    
    // Convert to angular coordinates
    double resolutionX = 960.0;
    double normalizedX = (targetX / resolutionX) - 0.5;
    double targetPointX = normalizedX * (TX_MAX - TX_MIN);
    
    // Log for debugging
    SmartDashboard.putNumber("AlgaeAdv_CenterX", centerX);
    SmartDashboard.putNumber("AlgaeAdv_TargetX", targetX);
    SmartDashboard.putNumber("AlgaeAdv_TargetY", targetY);
    SmartDashboard.putNumber("AlgaeAdv_TargetPointX", targetPointX);
    
    return targetPointX;
  }
  
  private double[] calculateControlValuesWithPrediction(TrackedObject obj, double targetX, double deltaTime) {
    // Map vision coordinates to robot control values
    double mappedTX = map(targetX, TX_MIN, TX_MAX, -1.0, 1.0);
    double mappedTY = map(obj.detection.tync, TY_MIN, TY_MAX, 0.0, 1.0);
    
    // Get object velocity for prediction
    double vx = obj.velocity.getX();
    double vy = obj.velocity.getY();
    
    // Calculate lookahead time based on object velocity
    double velocityMagnitude = Math.hypot(vx, vy);
    double lookaheadTime = Math.min(0.5, 0.1 + velocityMagnitude * 0.05);
    
    // Predict future position
    Translation2d predictedPos = obj.predictPosition(lastProcessTime + lookaheadTime);
    
    // Adjust mappedTX based on predicted horizontal movement
    double predictedTX = predictedPos.getX();
    double predictedMappedTX = map(predictedTX, TX_MIN, TX_MAX, -1.0, 1.0);
    
    // Calculate weights based on prediction confidence (more frames = more confidence)
    double predictionWeight = Math.min(0.5, obj.frameCount / 20.0);
    double currentWeight = 1.0 - predictionWeight;
    
    // Blend current position with predicted position
    double blendedMappedTX = (mappedTX * currentWeight) + (predictedMappedTX * predictionWeight);
    
    // Calculate control values for driving
    double forwardVelocity = -mappedTY * kForwardP;
    double rotationVelocity = -blendedMappedTX * kRotationP;
    
    // Log for debugging
    SmartDashboard.putNumber("AlgaeAdv_MappedTX", mappedTX);
    SmartDashboard.putNumber("AlgaeAdv_PredictedTX", predictedMappedTX);
    SmartDashboard.putNumber("AlgaeAdv_BlendedTX", blendedMappedTX);
    SmartDashboard.putNumber("AlgaeAdv_MappedTY", mappedTY);
    SmartDashboard.putNumber("AlgaeAdv_PredictionWeight", predictionWeight);
    
    return new double[] {forwardVelocity, rotationVelocity};
  }
  
  private boolean isBoxSquarish(RawDetection detection) {
    // Skip check if disabled
    if (!enableRectangularCheck) {
      return true;
    }
    
    // Check if bounding box has reasonable square-like proportions
    double[] dimensions = getBoundingBoxDimensions(detection);
    double width = dimensions[0];
    double height = dimensions[1];
    
    if (width == 0 || height == 0) {
      return false;
    }
    
    // Skip aspect ratio check for detections at the edge
    if (isAtCameraEdge(detection)) {
      return true;
    }
    
    // Check aspect ratio
    double aspectRatio = Math.max(width / height, height / width);
    return aspectRatio <= MAX_ASPECT_RATIO;
  }
  
  private boolean hasValidAreaRatio(RawDetection detection) {
    // Skip check if disabled
    if (!enableAreaRatioCheck) {
      return true;
    }
    
    // Check if object area ratio is appropriate for a ball
    double[] dimensions = getBoundingBoxDimensions(detection);
    double width = dimensions[0];
    double height = dimensions[1];
    
    double boxArea = width * height;
    if (boxArea == 0) {
      return false;
    }
    
    // Skip area ratio check for detections at the edge
    if (isAtCameraEdge(detection)) {
      return true;
    }
    
    // Calculate and check area ratio
    double objectArea = detection.ta * boxArea;
    double areaRatio = objectArea / boxArea;
    
    return areaRatio >= MIN_AREA_RATIO;
  }
  
  private double[] getBoundingBoxDimensions(RawDetection detection) {
    // Calculate width and height of the bounding box
    double minX = Math.min(
      Math.min(detection.corner0_X, detection.corner1_X),
      Math.min(detection.corner2_X, detection.corner3_X)
    );
    
    double maxX = Math.max(
      Math.max(detection.corner0_X, detection.corner1_X),
      Math.max(detection.corner2_X, detection.corner3_X)
    );
    
    double minY = Math.min(
      Math.min(detection.corner0_Y, detection.corner1_Y),
      Math.min(detection.corner2_Y, detection.corner3_Y)
    );
    
    double maxY = Math.max(
      Math.max(detection.corner0_Y, detection.corner1_Y),
      Math.max(detection.corner2_Y, detection.corner3_Y)
    );
    
    return new double[] {maxX - minX, maxY - minY, minX, maxX, minY, maxY};
  }
  
  private boolean isAtCameraEdge(RawDetection detection) {
    // Check if detection is at the edge of camera view
    double[] dimensions = getBoundingBoxDimensions(detection);
    double minX = dimensions[2];
    double maxX = dimensions[3];
    double minY = dimensions[4];
    double maxY = dimensions[5];
    
    double[] resolution = getLimelightResolution();
    double resolutionX = resolution[0];
    double resolutionY = resolution[1];
    
    return (minX < EDGE_MARGIN || maxX > resolutionX - EDGE_MARGIN || 
            minY < EDGE_MARGIN || maxY > resolutionY - EDGE_MARGIN);
  }
  
  private double[] getLimelightResolution() {
    // Get current limelight resolution from JSON data
    double resolutionX = 320.0;
    double resolutionY = 240.0;
    
    String jsonData = LimelightHelpers.getJSONDump(limelightName);
    if (jsonData != null && !jsonData.isEmpty()) {
      try {
        int widthIndex = jsonData.indexOf("\"width\":");
        int heightIndex = jsonData.indexOf("\"height\":");
        if (widthIndex >= 0 && heightIndex >= 0) {
          String widthStr = jsonData.substring(widthIndex + 8, jsonData.indexOf(",", widthIndex));
          String heightStr = jsonData.substring(heightIndex + 9, jsonData.indexOf(",", heightIndex));
          resolutionX = Double.parseDouble(widthStr);
          resolutionY = Double.parseDouble(heightStr);
        }
      } catch (Exception e) {
        // Use default resolution if parsing fails
      }
    }
    
    return new double[] {resolutionX, resolutionY};
  }
  
  private double map(double value, double inMin, double inMax, double outMin, double outMax) {
    // Map a value from one range to another
    value = Math.max(inMin, Math.min(inMax, value));
    return outMin + (value - inMin) * (outMax - outMin) / (inMax - inMin);
  }
  
  private void displayTrackingData() {
    if (selectedObject != null) {
      // Display selected object data
      SmartDashboard.putNumber("AlgaeAdv_SelectedID", selectedObject.id);
      SmartDashboard.putString("AlgaeAdv_Algorithm", currentAlgorithm.toString());
      SmartDashboard.putNumber("AlgaeAdv_FrameCount", selectedObject.frameCount);
      SmartDashboard.putNumber("AlgaeAdv_VelocityX", selectedObject.velocity.getX());
      SmartDashboard.putNumber("AlgaeAdv_VelocityY", selectedObject.velocity.getY());
      SmartDashboard.putNumber("AlgaeAdv_Distance", selectedObject.distance);
      
      // Calculate and display speed
      double speedInPixels = Math.hypot(selectedObject.velocity.getX(), selectedObject.velocity.getY());
      SmartDashboard.putNumber("AlgaeAdv_Speed", speedInPixels);
      
      // Display position and detection
      RawDetection detection = selectedObject.detection;
      SmartDashboard.putNumber("AlgaeAdv_Selected_TX", detection.txnc);
      SmartDashboard.putNumber("AlgaeAdv_Selected_TY", detection.tync);
      SmartDashboard.putNumber("AlgaeAdv_Selected_Area", detection.ta);
    } else {
      // Clear dashboard values when no object is selected
      SmartDashboard.putNumber("AlgaeAdv_SelectedID", -1);
      SmartDashboard.putString("AlgaeAdv_Algorithm", "None");
      SmartDashboard.putNumber("AlgaeAdv_FrameCount", 0);
      SmartDashboard.putNumber("AlgaeAdv_VelocityX", 0);
      SmartDashboard.putNumber("AlgaeAdv_VelocityY", 0);
      SmartDashboard.putNumber("AlgaeAdv_Distance", 0);
      SmartDashboard.putNumber("AlgaeAdv_Speed", 0);
      SmartDashboard.putNumber("AlgaeAdv_Selected_TX", 0);
      SmartDashboard.putNumber("AlgaeAdv_Selected_TY", 0);
      SmartDashboard.putNumber("AlgaeAdv_Selected_Area", 0);
    }
    
    // Display total number of tracked objects
    SmartDashboard.putNumber("AlgaeAdv_ObjectCount", trackedObjects.size());
  }
  
  // Estimate distance to object using several methods
  private double estimateDistance(RawDetection detection) {
    // Method 1: Use vertical angle (ty) and known height
    double ty = detection.tync;
    double angleToTarget = Math.toRadians(ty);
    double distance1 = CAMERA_HEIGHT_METERS / 
                      Math.tan(CAMERA_PITCH_RADIANS + angleToTarget);
    
    // Method 2: Use object size
    double[] dimensions = getBoundingBoxDimensions(detection);
    double objectWidth = dimensions[0];  // Width in pixels
    double resolutionX = getLimelightResolution()[0];
    double fovRadians = Math.toRadians(TX_MAX - TX_MIN);
    
    // Calculate angular size and distance
    double angularSize = (objectWidth / resolutionX) * fovRadians;
    double distance2 = ALGAE_DIAMETER_METERS / (2 * Math.tan(angularSize / 2));
    
    // Blend the two methods
    // More weight to method 1 when object is directly in front
    // More weight to method 2 when at edge of frame
    double distanceFromCenter = Math.abs(detection.txnc) / TX_MAX;
    double weight2 = Math.min(1.0, 0.3 + distanceFromCenter * 0.7);
    double weight1 = 1.0 - weight2;
    
    double distance = (distance1 * weight1) + (distance2 * weight2);
    
    // Ensure reasonable values
    return Math.max(0.2, Math.min(10.0, distance));
  }
  
  private double getCurrentTime() {
    return System.currentTimeMillis() / 1000.0;
  }
  
  // Public methods to configure algorithm and settings
  public void setSelectionAlgorithm(SelectionAlgorithm algorithm) {
    currentAlgorithm = algorithm;
  }
  
  public void setRectangularCheckEnabled(boolean enabled) {
    enableRectangularCheck = enabled;
  }
  
  public void setAreaRatioCheckEnabled(boolean enabled) {
    enableAreaRatioCheck = enabled;
  }
  
  public void setPIDEnabled(boolean enabled) {
    usePIDController = enabled;
    
    // Reset PID controllers when switching
    if (enabled) {
      rotationPID.reset();
      forwardPID.reset();
    }
  }
  
  // Methods to configure PID values
  public void setRotationPID(double p, double i, double d) {
    rotationPID.setPID(p, i, d);
  }
  
  public void setForwardPID(double p, double i, double d) {
    forwardPID.setPID(p, i, d);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain when command ends
    selectedObject = null;
    m_swerve.setControl(roboDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    // End only when parent commands end it
    return false;
  }
} 