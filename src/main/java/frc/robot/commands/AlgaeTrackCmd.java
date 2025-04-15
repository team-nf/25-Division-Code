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
  
  // Points array to store 4 points for line intersection calculation
  // Format: [point0_x, point0_y, point1_x, point1_y, point2_x, point2_y, point3_x, point3_y]
  private static final double[] linePoints = new double[8];
  
  // Subsystem and limelight configuration
  private CommandSwerveDrivetrain m_swerve;
  private final String limelightName = "limelight-obj";
  
  // Algorithm selection and filtering flags
  private static SelectionAlgorithm CURRENT_SELECTION_ALGORITHM = SelectionAlgorithm.LOWEST_FIRST;
  private boolean enableRectangularCheck = true;
  private boolean enableAreaRatioCheck = true;
  
  // Target point in bounding box using Cartesian coordinates (-1 to 1, -1 to 1)
  // where (0,0) is center, (-1,0) is left center, (1,0) is right center
  // (0,1) is top center, (0,-1) is bottom center
  private static double TARGET_OFFSET_X = -0.8;  // Default: 80% toward left edge from center
  private static double TARGET_OFFSET_Y = 0.0;   // Default: vertical center
  
  // Constants
  private static final double TX_MIN = -29.8;
  private static final double TX_MAX = 29.8;
  private static final double TY_MIN = -24.85;
  private static final double TY_MAX = 24.85;

  private static final double kForwardP = 3.0;
  private static final double kForwardD = 0.0;
  private static final double kForwardI = 0.0;

  private static final double kRotationP = 1.2;
  private static final double kRotationD = 0.0;
  private static final double kRotationI = 0.0;

  private static final double MAX_ASPECT_RATIO = 1.3;
  private static final double MIN_AREA_RATIO = 0.65;
  private static final double EDGE_MARGIN = 5.0;
  
  // Slew rate limiters (units per second)
  private static final double FORWARD_RATE_LIMIT = 3.0;  // Meters per second per second
  private static final double ROTATION_RATE_LIMIT = 6.0; // Radians per second per second
  private final SlewRateLimiter forwardLimiter;
  private final SlewRateLimiter rotationLimiter;

  private static final double HEIGHT_THRESHOLD_PERCENT = 0.9;
  
  // TY koordinatının yorumlanması:
  // TY_MIN: Ekranın alt kısmı (negatif değer, genellikle -24.85)
  // TY_MAX: Ekranın üst kısmı (pozitif değer, genellikle +24.85)
  // 
  // Doğru hesaplama: Ekranın en altından (TY_MIN) başlayarak 
  // toplam yüksekliğin (TY_MAX - TY_MIN) yüzde 60'ı kadar yukarı git
  private static final double TY_THRESHOLD = TY_MIN + ((TY_MAX - TY_MIN) * HEIGHT_THRESHOLD_PERCENT);
  
  // Virtual screen points for angle calculation
  private static final double VIRTUAL_BOTTOM_X = 0.0; // Default center X is 0 in angular coordinates
  private static final double VIRTUAL_BOTTOM_Y = -180.0; // Default point below the screen

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
    
    double maxSpeed = MetersPerSecond.of(1.5).in(MetersPerSecond);
    roboDrive = new SwerveRequest.RobotCentric()
      .withDeadband(maxSpeed * 0.035)
      .withDriveRequestType(DriveRequestType.Velocity);
  }

  @Override
  public void initialize() {
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
      double angleToTarget = calculateAngleToTarget(targetPoint);
      double mappedTY = 1 - map(targetPoint[1], TY_MIN, TY_MAX, 0.0, 1.0);
      
      // Use PID controllers
      double forwardVelocity = forwardPID.calculate(mappedTY, 0);
      SmartDashboard.putNumber("Algae_forwardVel", forwardVelocity);

      double rotationVelocity = rotationPID.calculate(angleToTarget, 0); // Target is center (0 angle)
      // double rotationVelocity = rotationPID.calculate(mappedTX, 0); // Target is center (0 angle)
      
      // Apply slew rate limiters for smoother motion
      double smoothForwardVelocity = forwardLimiter.calculate(forwardVelocity);
      // double smoothRotationVelocity = rotationLimiter.calculate(rotationVelocity);

      // Drive robot using calculated velocities with rate limiting applied
      m_swerve.setControl(
        roboDrive
          .withVelocityX(-smoothForwardVelocity)
          // .withVelocityX(0.0)
          .withVelocityY(0.0)
          .withRotationalRate(-rotationVelocity)
          // .withRotationalRate(0)
      );
    } else {
      // If no target, smoothly stop the robot
      double smoothForwardVelocity = forwardLimiter.calculate(0);

      if (Math.abs(smoothForwardVelocity) > 0.01) {
        m_swerve.setControl(
          roboDrive
            .withVelocityX(-smoothForwardVelocity)
            // .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.0)
            // .withRotationalRate(0.0)
        );
      }
    }
  }

  private void processLimelightDetections() {
    boolean hasTarget = LimelightHelpers.getTV(limelightName);
    
    if (hasTarget) {
      RawDetection[] detections = LimelightHelpers.getRawDetections(limelightName);
      
      if (detections != null && detections.length > 0) {
        // //Smart Dashboard.putNumber("LL_DetectionCount", detections.length);
        
        // Select algae based on algorithm
        switch (CURRENT_SELECTION_ALGORITHM) {
          case LOWEST_FIRST:
            selectLowestAlgae(detections);
            break;
          default:
            selectLowestAlgae(detections);
            break;
        }
      } else {
        // //Smart Dashboard.putNumber("LL_DetectionCount", 0);
        selectedAlgae = null;
      }
    } else {
      // //Smart Dashboard.putNumber("LL_DetectionCount", 0);
      selectedAlgae = null;
    }
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
    double targetX = centerX + (TARGET_OFFSET_X * width / 2.0);
    double targetY = centerY + (TARGET_OFFSET_Y * height / 2.0);

    // Ensure target point is within bounds
    targetX = Math.max(minX, Math.min(maxX, targetX));
    targetY = Math.max(minY, Math.min(maxY, targetY));

    double resolutionX = 960.0;
    double resolutionY = 740.0;
    
    double normalizedX = (targetX / resolutionX) - 0.5;
    double normalizedY = (targetY / resolutionY) - 0.5;
    
    double targetPointX = normalizedX * (TX_MAX - TX_MIN);
    double targetPointY = normalizedY * (TY_MAX - TY_MIN); 

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
    // //Smart Dashboard.putNumber("Algae_AreaRatio", areaRatio);
    
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
      // Skip if the detection is too high on the screen (top 40%)
      if (detection.tync > TY_THRESHOLD) {
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
  
  
  private double map(double value, double inMin, double inMax, double outMin, double outMax) {
    // Map a value from one range to another
    value = Math.max(inMin, Math.min(inMax, value));
    return outMin + (value - inMin) * (outMax - outMin) / (inMax - inMin);
  }

  // Public methods to configure algorithm and settings
  public void setSelectionAlgorithm(SelectionAlgorithm algorithm) {
    CURRENT_SELECTION_ALGORITHM = algorithm;
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

  private double calculateAngleToTarget(double[] targetPoint) {
    try {
      // Target coordinates
      double targetX = targetPoint[0];
      double targetY = targetPoint[1];
      
      // Use global virtual reference point
      // This gives more stable angles and prevents extreme values
      
      // Calculate the horizontal and vertical differences
      double deltaX = targetX - VIRTUAL_BOTTOM_X;
      double deltaY = VIRTUAL_BOTTOM_Y - targetY;
      
      // Safety check for division by zero or very small values
      if (Math.abs(deltaY) < 0.001) {
        // Avoid division by zero by setting a minimum value
        deltaY = (deltaY >= 0) ? 0.001 : -0.001;
      }
      
      double angleRadians = 0;
      double angleDegrees = 0;
      
      // Calculate the angle using arctangent
      angleRadians = Math.atan2(deltaX, deltaY);

      angleDegrees = 180 + Math.toDegrees(angleRadians);
      if (angleDegrees > 180)  {
        angleDegrees = angleDegrees - 360 ;
      }
      
      // Check for NaN or Infinity
      if (Double.isNaN(angleDegrees) || Double.isInfinite(angleDegrees)) {
        angleDegrees = (deltaX > 0) ? 45.0 : -45.0; // Default to 45 degrees in the direction of deltaX
      }
      
      // Normalize to [-1, 1] for PID controller
      double normalizedAngle = angleDegrees / 45.0;
      normalizedAngle = Math.max(-1.0, Math.min(1.0, normalizedAngle));

      return normalizedAngle;
    } catch (Exception e) {
      // Error handling
      System.out.println("Error in angle calculation: " + e.getMessage());
      return 0.0; // Default safe value
    }
  }
} 