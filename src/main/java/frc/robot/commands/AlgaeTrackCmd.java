// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* Tracks algae objects using Limelight vision and drives toward them */
public class AlgaeTrackCmd extends Command {
  // Selection algorithm enum
  public enum SelectionAlgorithm {
    LOWEST_FIRST,        // Select the lowest algae in frame (highest ty value)
    LOWER_CENTER_CLOSEST, // Select algae closest to bottom center
    BIGGEST              // Select largest algae by area
  }
  
  // Subsystem and limelight configuration
  private CommandSwerveDrivetrain m_swerve;
  private final String limelightName = "limelight-obj";
  
  // Algorithm selection and filtering flags
  private static SelectionAlgorithm currentAlgorithm = SelectionAlgorithm.LOWEST_FIRST;
  private boolean enableRectangularCheck = true;
  private boolean enableAreaRatioCheck = true;
  
  // Target point in bounding box using Cartesian coordinates (-1 to 1, -1 to 1)
  // where (0,0) is center, (-1,0) is left center, (1,0) is right center
  // (0,1) is top center, (0,-1) is bottom center
  private static double targetOffsetX = -0.8;  // Default: 80% toward left edge from center
  private static double targetOffsetY = 0.0;   // Default: vertical center
  
  // Constants
  private static final double TX_MIN = -29.8;
  private static final double TX_MAX = 29.8;
  private static final double TY_MIN = -24.85;
  private static final double TY_MAX = 24.85;

  private static final double kForwardP = 5.0;
  private static final double kForwardD = 0.0;
  private static final double kForwardI = 0.0;

  private static final double kRotationP = 1.5;
  private static final double kRotationD = 0.0;
  private static final double kRotationI = 0.0;

  private static final double MAX_ASPECT_RATIO = 1.3;
  private static final double MIN_AREA_RATIO = 0.65;
  private static final double EDGE_MARGIN = 5.0;
  
  // Slew rate limiters (units per second)
  private static final double FORWARD_RATE_LIMIT = 2.0;  // Meters per second per second
  private static final double ROTATION_RATE_LIMIT = 6.0; // Radians per second per second
  private final SlewRateLimiter forwardLimiter;
  private final SlewRateLimiter rotationLimiter;

  
  // Height threshold - Only select objects in the bottom 60% of screen
  private static final double HEIGHT_THRESHOLD_PERCENT = 0.6; // 60% from bottom of screen
  private static final double TY_THRESHOLD = TY_MIN + ((TY_MAX - TY_MIN) * (1 - HEIGHT_THRESHOLD_PERCENT));

  // PID controllers
  private final PIDController rotationPID;
  private final PIDController forwardPID;
  
  // Drive configuration
  private final SwerveRequest.RobotCentric roboDrive;
  
  // Current tracking state
  private RawDetection selectedAlgae = null;

  public AlgaeTrackCmd(CommandSwerveDrivetrain swerve) {
    m_swerve = swerve;
    addRequirements(m_swerve);
    
    // Create PID controllers
    rotationPID = new PIDController(kRotationP, kRotationI, kRotationD);
    forwardPID = new PIDController(kForwardP, kForwardI, kForwardD);
    
    // Create slew rate limiters
    forwardLimiter = new SlewRateLimiter(FORWARD_RATE_LIMIT);
    rotationLimiter = new SlewRateLimiter(ROTATION_RATE_LIMIT);
    
    // Set input range for rotation PID (helps with wrap-around)
    rotationPID.enableContinuousInput(-1.0, 1.0);
    
    double maxSpeed = MetersPerSecond.of(2).in(MetersPerSecond);
    roboDrive = new SwerveRequest.RobotCentric()
      .withDeadband(maxSpeed * 0.035)
      .withDriveRequestType(DriveRequestType.Velocity);
  }

  @Override
  public void initialize() {
    // Initialize SmartDashboard values
    SmartDashboard.putNumber("Algae_ForwardVelocity", 0);
    SmartDashboard.putNumber("Algae_RotationVelocity", 0);
    SmartDashboard.putNumber("LL_TX", 0);
    SmartDashboard.putNumber("LL_TY", 0);
    SmartDashboard.putNumber("LL_TA", 0);
    SmartDashboard.putString("LL_DetectorClass", "");
    SmartDashboard.putNumber("LL_DetectorClassIndex", 0);
    SmartDashboard.putString("Algae_Algorithm", currentAlgorithm.toString());
    SmartDashboard.putBoolean("Algae_EnableRectCheck", enableRectangularCheck);
    SmartDashboard.putBoolean("Algae_EnableAreaCheck", enableAreaRatioCheck);
    SmartDashboard.putNumber("Algae_TargetOffsetX", targetOffsetX);
    SmartDashboard.putNumber("Algae_TargetOffsetY", targetOffsetY);
    SmartDashboard.putNumber("Algae_AreaRatio", 0);
    SmartDashboard.putNumber("Algae_LeftX", 0);
    SmartDashboard.putNumber("Algae_TargetX", 0);
    SmartDashboard.putNumber("Algae_TargetY", 0);
    SmartDashboard.putNumber("Algae_MappedTX", 0);
    SmartDashboard.putNumber("Algae_MappedTY", 0);
    SmartDashboard.putBoolean("LL_HasTarget", false);
    SmartDashboard.putNumber("LL_DetectionCount", 0);
    SmartDashboard.putNumber("Algae_Selected_ClassID", -1);
    SmartDashboard.putNumber("Algae_Selected_TX", 0);
    SmartDashboard.putNumber("Algae_Selected_TY", 0);
    SmartDashboard.putNumber("Algae_Selected_Area", 0);
    SmartDashboard.putNumber("Algae_BB_w", 0);
    SmartDashboard.putNumber("Algae_BB_h", 0);
    SmartDashboard.putNumber("Algae_SmoothForwardVelocity", 0);
    SmartDashboard.putNumber("Algae_SmoothRotationVelocity", 0);
    SmartDashboard.putNumber("Algae_TY_Threshold", TY_THRESHOLD);

    // Set limelight pipeline for neural detection and reset tracking
    LimelightHelpers.setPipelineIndex(limelightName, 0);
    selectedAlgae = null;
    
    // Reset PID controllers and slew rate limiters
    rotationPID.reset();
    forwardPID.reset();
    // Reset slew rate limiters to 0
    forwardLimiter.reset(0);
    rotationLimiter.reset(0);
  }

  @Override
  public void execute() {
    // Process detections and drive toward target if found
    processLimelightDetections();
    
    if (selectedAlgae != null) {
      // Calculate target point
      double[] targetPoint = calculateTargetPoint(selectedAlgae);
      
      // Calculate angle to target using the target point
      // double angleToTarget = calculateAngleToTarget(targetPoint);
      
      // Calculate Y error for forward movement
      double mappedTX = map(targetPoint[0], TX_MIN, TX_MAX, -1.0, 1.0);
      double mappedTY = map(-targetPoint[1], TY_MIN, TY_MAX, 0.0, 1.0);
      
      // Use PID controllers
      // double rotationVelocity = rotationPID.calculate(angleToTarget, 0); // Target is center (0 angle)
      double rotationVelocity = rotationPID.calculate(mappedTX, 0); // Target is center (0 angle)
      double forwardVelocity = forwardPID.calculate(mappedTY, 0);   // Target is 0 (minimum distance)
      
      // Apply slew rate limiters for smoother motion
      double smoothForwardVelocity = forwardLimiter.calculate(forwardVelocity);
      double smoothRotationVelocity = rotationLimiter.calculate(rotationVelocity);
      
      // Log control values
      SmartDashboard.putNumber("Algae_AngleToTarget", angleToTarget);
      SmartDashboard.putNumber("Algae_ForwardVelocity", forwardVelocity);
      SmartDashboard.putNumber("Algae_RotationVelocity", rotationVelocity);
      SmartDashboard.putNumber("Algae_SmoothForwardVelocity", smoothForwardVelocity);
      SmartDashboard.putNumber("Algae_SmoothRotationVelocity", smoothRotationVelocity);
      
      // Drive robot using calculated velocities with rate limiting applied
      m_swerve.setControl(
        roboDrive
          // .withVelocityX(-smoothForwardVelocity) // Forward velocity
          .withVelocityX(0.0) // Forward velocity
          .withVelocityY(0.0)                    // No side-to-side motion
          .withRotationalRate(-smoothRotationVelocity) // Rotation velocity
      );
    } else {
      // If no target, smoothly stop the robot
      double smoothForwardVelocity = forwardLimiter.calculate(0);
      double smoothRotationVelocity = rotationLimiter.calculate(0);
      
      if (Math.abs(smoothForwardVelocity) > 0.01 || Math.abs(smoothRotationVelocity) > 0.01) {
        m_swerve.setControl(
          roboDrive
            .withVelocityX(-smoothForwardVelocity)
            .withVelocityY(0.0)
            .withRotationalRate(smoothRotationVelocity)
        );
      }
    }
  }

  private void processLimelightDetections() {
    // Get and process vision data, selecting the best target
    boolean hasTarget = LimelightHelpers.getTV(limelightName);
    SmartDashboard.putBoolean("LL_HasTarget", hasTarget);
    
    if (hasTarget) {
      updateLimelightData();
      RawDetection[] detections = LimelightHelpers.getRawDetections(limelightName);
      
      if (detections != null && detections.length > 0) {
        SmartDashboard.putNumber("LL_DetectionCount", detections.length);
        
        // Select algae based on algorithm
        switch (currentAlgorithm) {
          case LOWEST_FIRST:
            selectLowestAlgae(detections);
            break;
          case LOWER_CENTER_CLOSEST:
            selectClosestToBottomCenter(detections);
            break;
          case BIGGEST:
            selectBiggestAlgae(detections);
            break;
          default:
            selectLowestAlgae(detections);
            break;
        }
      } else {
        SmartDashboard.putNumber("LL_DetectionCount", 0);
        selectedAlgae = null;
      }
    } else {
      SmartDashboard.putNumber("LL_DetectionCount", 0);
      selectedAlgae = null;
    }
    
    displaySelectedAlgae();
  }
  
  private void updateLimelightData() {
    // Update SmartDashboard with basic Limelight data
    SmartDashboard.putNumber("LL_TX", LimelightHelpers.getTX(limelightName));
    SmartDashboard.putNumber("LL_TY", LimelightHelpers.getTY(limelightName));
    SmartDashboard.putNumber("LL_TA", LimelightHelpers.getTA(limelightName));
    SmartDashboard.putString("LL_DetectorClass", LimelightHelpers.getDetectorClass(limelightName));
    SmartDashboard.putNumber("LL_DetectorClassIndex", LimelightHelpers.getDetectorClassIndex(limelightName));
    
    // Display algorithm and filter settings
    SmartDashboard.putString("Algae_Algorithm", currentAlgorithm.toString());
    SmartDashboard.putBoolean("Algae_EnableRectCheck", enableRectangularCheck);
    SmartDashboard.putBoolean("Algae_EnableAreaCheck", enableAreaRatioCheck);
    SmartDashboard.putNumber("Algae_TargetOffsetX", targetOffsetX);
    SmartDashboard.putNumber("Algae_TargetOffsetY", targetOffsetY);
    SmartDashboard.putNumber("Algae_TY_Threshold", TY_THRESHOLD);
  }
  
  private double[] calculateTargetPoint(RawDetection detection) {
    // Calculate bounding box dimensions and center
    double[] dimensions = getBoundingBoxDimensions(detection);
    double width = dimensions[0];
    double height = dimensions[1];
    double minX = dimensions[2];
    double maxX = dimensions[3];
    double minY = dimensions[4];
    double maxY = dimensions[5];
    
    // bunlar -30 +30 arası değerler
    // bounding boxın orta kordinatı
    double centerX = (minX + maxX) / 2.0;
    double centerY = (minY + maxY) / 2.0;
    
    // bu da bizim hedef noktanın -30 +30 arası değeri
    double targetX = centerX + (targetOffsetX * width / 2.0);
    double targetY = centerY + (targetOffsetY * height / 2.0);

    // Ensure target point is within bounds
    targetX = Math.max(minX, Math.min(maxX, targetX));
    targetY = Math.max(minY, Math.min(maxY, targetY));

    // double[] resolution = getLimelightResolution();
    // double resolutionX = resolution[0];
    // double resolutionY = resolution[1];
    double resolutionX = 960.0;
    double resolutionY = 740.0;
    
    double normalizedX = (targetX / resolutionX) - 0.5;
    double normalizedY = (targetY / resolutionY) - 0.5;
    
    double targetPointX = normalizedX * (TX_MAX - TX_MIN);
    double targetPointY = normalizedY * (TY_MAX - TY_MIN); 
    
    // Log for debugging
    SmartDashboard.putNumber("Algae_BB_w", width);
    SmartDashboard.putNumber("Algae_BB_h", height);
    SmartDashboard.putNumber("Algae_TargetX", targetX);
    SmartDashboard.putNumber("Algae_TargetY", targetY);
    SmartDashboard.putNumber("Algae_TargetPointX", targetPointX);
    SmartDashboard.putNumber("Algae_TargetPointY", targetPointY);
    
    return new double[] {targetPointX , targetPointY};
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
    SmartDashboard.putNumber("Algae_AreaRatio", areaRatio);
    
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
    // return new double[] {960.0, 720.0};

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
  
  private void selectLowestAlgae(RawDetection[] detections) {
    // Select the lowest algae in frame (highest ty value) that passes validation
    selectedAlgae = null;
    double highestTY = -Double.MAX_VALUE;
    
    for (RawDetection detection : detections) {
      // Skip if the detection is too high on the screen (top 30%)
      if (detection.tync < TY_THRESHOLD) {
        continue;
      }
      
      if (!isBoxSquarish(detection) || !hasValidAreaRatio(detection)) {
        continue;
      }
      
      if (detection.tync > highestTY) {
        highestTY = detection.tync;
        selectedAlgae = detection;
      }
    }
  }
  
  private void selectClosestToBottomCenter(RawDetection[] detections) {
    // Select algae closest to bottom center that passes validation
    selectedAlgae = null;
    double closestDistance = Double.MAX_VALUE;
    
    for (RawDetection detection : detections) {
      // Skip if the detection is too high on the screen (top 30%)
      if (detection.tync < TY_THRESHOLD) {
        continue;
      }
      
      if (!isBoxSquarish(detection) || !hasValidAreaRatio(detection)) {
        continue;
      }
      
      // Calculate distance to bottom center (approximately (0, TY_MIN))
      double distToBottomCenter = Math.sqrt(
          Math.pow(detection.txnc, 2) + 
          Math.pow(detection.tync - TY_MIN, 2));
      
      if (distToBottomCenter < closestDistance) {
        closestDistance = distToBottomCenter;
        selectedAlgae = detection;
      }
    }
  }
  
  private void selectBiggestAlgae(RawDetection[] detections) {
    // Select the largest algae (by area) that passes validation
    selectedAlgae = null;
    double largestArea = -Double.MAX_VALUE;
    
    for (RawDetection detection : detections) {
      // Skip if the detection is too high on the screen (top 30%)
      if (detection.tync < TY_THRESHOLD) {
        continue;
      }
      
      if (!isBoxSquarish(detection) || !hasValidAreaRatio(detection)) {
        continue;
      }
      
      if (detection.ta > largestArea) {
        largestArea = detection.ta;
        selectedAlgae = detection;
      }
    }
  }
  
  private double map(double value, double inMin, double inMax, double outMin, double outMax) {
    // Map a value from one range to another
    value = Math.max(inMin, Math.min(inMax, value));
    return outMin + (value - inMin) * (outMax - outMin) / (inMax - inMin);
  }
  
  private void displaySelectedAlgae() {
    // Update dashboard with selected algae information
    if (selectedAlgae != null) {
      SmartDashboard.putString("Algae_SelectionAlgorithm", currentAlgorithm.toString());
      SmartDashboard.putNumber("Algae_Selected_ClassID", selectedAlgae.classId);
      SmartDashboard.putNumber("Algae_Selected_TX", selectedAlgae.txnc);
      SmartDashboard.putNumber("Algae_Selected_TY", selectedAlgae.tync);
      SmartDashboard.putNumber("Algae_Selected_Area", selectedAlgae.ta);
    } else {
      SmartDashboard.putString("Algae_SelectionAlgorithm", "None");
      SmartDashboard.putNumber("Algae_Selected_ClassID", -1);
      SmartDashboard.putNumber("Algae_Selected_TX", 0);
      SmartDashboard.putNumber("Algae_Selected_TY", 0);
      SmartDashboard.putNumber("Algae_MappedTX", 0);
      SmartDashboard.putNumber("Algae_MappedTY", 0);
      SmartDashboard.putNumber("Algae_RotationVelocity", 0);
      SmartDashboard.putNumber("Algae_ForwardVelocity", 0);
    }
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
    selectedAlgae = null;
    // m_swerve.setControl(roboDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    // End only when parent commands end it
    
    return false;
  }

  /**
   * Calculates the angle to the target using a simple right triangle approach.
   * Forms a right triangle with vertices at:
   * 1. The target point (x, y)
   * 2. A virtual point below the screen's bottom center
   * 3. A point directly below the target at the same height as the virtual point
   * 
   * @param targetPoint The target point [x, y] in screen space
   * @return The angle to the target in degrees (-1 to 1 range, suitable for PID)
   */
  // private double calculateAngleToTarget(double[] targetPoint) {
  //   // Target coordinates
  //   double targetX = targetPoint[0];
  //   double targetY = targetPoint[1];
    
  //   // Create a virtual reference point below the screen
  //   // This gives more stable angles and prevents extreme values
  //   double bottomCenterX = 0; // Center X is 0 in angular coordinates
    
  //   // Instead of using the screen bottom (TY_MIN),
  //   // use a point further down by half the screen height
  //   double screenHeight = TY_MAX - TY_MIN;
  //   double virtualBottomY = TY_MIN - (screenHeight / 2.0);
    
  //   // Calculate the horizontal and vertical differences
  //   double deltaX = targetX - bottomCenterX;
  //   double deltaY = virtualBottomY - targetY; // Using the virtual point below the screen
    
  //   // Safety check for division by zero or very small values
  //   if (Math.abs(deltaY) < 0.001) {
  //     // Avoid division by zero by setting a minimum value
  //     deltaY = (deltaY >= 0) ? 0.001 : -0.001;
  //   }
    
  //   double angleRadians = 0;
  //   double angleDegrees = 0;
    
  //   try {
  //     // Calculate the angle using arctangent
  //     angleRadians = Math.atan2(deltaX, deltaY);
  //     angleDegrees = Math.toDegrees(angleRadians);
  //   } catch (Exception e) {
  //     // If any error occurs, default to a safe value
  //     System.out.println("Error in angle calculation: " + e.getMessage());
  //     angleDegrees = (deltaX > 0) ? 45.0 : -45.0; // Default to 45 degrees in the direction of deltaX
  //   }
    
  //   // Check for NaN or Infinity
  //   if (Double.isNaN(angleDegrees) || Double.isInfinite(angleDegrees)) {
  //     angleDegrees = (deltaX > 0) ? 45.0 : -45.0; // Default to 45 degrees in the direction of deltaX
  //   }
    
  //   // Normalize to [-1, 1] for PID controller
  //   double normalizedAngle = angleDegrees / 90.0;
  //   normalizedAngle = Math.max(-1.0, Math.min(1.0, normalizedAngle));
    
  //   // Log for debugging
  //   SmartDashboard.putNumber("Algae_DeltaX", deltaX);
  //   SmartDashboard.putNumber("Algae_DeltaY", deltaY);
  //   SmartDashboard.putNumber("Algae_AngleDegrees", angleDegrees);
  //   SmartDashboard.putNumber("Algae_VirtualBottomY", virtualBottomY);
    
  //   return normalizedAngle;
  // }
} 