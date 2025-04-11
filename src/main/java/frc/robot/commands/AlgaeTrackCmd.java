// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
  private boolean usePIDController = true;
  
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
  private static final double kForwardP = 3.0;
  private static final double kRotationP = 1.8;
  private static final double MAX_ASPECT_RATIO = 1.3;
  private static final double MIN_AREA_RATIO = 0.65;
  private static final double EDGE_MARGIN = 5.0;

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
    rotationPID = new PIDController(kRotationP, 0, 0);
    forwardPID = new PIDController(kForwardP, 0, 0);
    
    // Set input range for rotation PID (helps with wrap-around)
    rotationPID.enableContinuousInput(-1.0, 1.0);
    
    double maxSpeed = MetersPerSecond.of(1).in(MetersPerSecond);
    roboDrive = new SwerveRequest.RobotCentric()
      .withDeadband(maxSpeed * 0.035)
      .withDriveRequestType(DriveRequestType.Velocity);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("Algae_ForwardVelocity", 0);
    SmartDashboard.putNumber("Algae_RotationVelocity", 0);
    SmartDashboard.putBoolean("Algae_UsingPID", false);
    SmartDashboard.putBoolean("LL_HasTarget", false);

    // Set limelight pipeline for neural detection and reset tracking
    LimelightHelpers.setPipelineIndex(limelightName, 0);
    selectedAlgae = null;
    
    // Reset PID controllers
    rotationPID.reset();
    forwardPID.reset();
  }

  @Override
  public void execute() {
    // Process detections and drive toward target if found
    processLimelightDetections();
    
    if (selectedAlgae != null) {
      // Calculate drive values
      double targetPointX = calculateTargetPoint(selectedAlgae);
      
      double forwardVelocity;
      double rotationVelocity;
      
      if (usePIDController) {
        // Use PID controllers
        double mappedTX = map(targetPointX, TX_MIN, TX_MAX, -1.0, 1.0);
        double mappedTY = map(selectedAlgae.tync, TY_MIN, TY_MAX, 0.0, 1.0);
        
        rotationVelocity = -rotationPID.calculate(mappedTX, 0); // Target is center (0)
        forwardVelocity = -forwardPID.calculate(mappedTY, 0);   // Target is 0 (minimum distance)
      } else {
        // Use simple P controller
        double[] controlValues = calculateControlValues(targetPointX, selectedAlgae.tync);
        forwardVelocity = controlValues[0];
        rotationVelocity = controlValues[1];
      }
      
      // Log control values
      SmartDashboard.putNumber("Algae_ForwardVelocity", forwardVelocity);
      SmartDashboard.putNumber("Algae_RotationVelocity", rotationVelocity);
      SmartDashboard.putBoolean("Algae_UsingPID", usePIDController);
      
      // Drive robot using calculated velocities
      m_swerve.setControl(
        roboDrive
          .withVelocityX(-forwardVelocity) // Forward velocity
          .withVelocityY(0.0)               // No side-to-side motion
          .withRotationalRate(rotationVelocity) // Rotation velocity
      );
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
    SmartDashboard.putNumber("Algae_LeftX", minX);
    SmartDashboard.putNumber("Algae_CenterX", centerX);
    SmartDashboard.putNumber("Algae_TargetX", targetX);
    SmartDashboard.putNumber("Algae_TargetY", targetY);
    SmartDashboard.putNumber("Algae_TargetPointX", targetPointX);
    SmartDashboard.putNumber("Algae_TargetOffsetX", targetOffsetX);
    SmartDashboard.putNumber("Algae_TargetOffsetY", targetOffsetY);
    
    return targetPointX;
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
      if (!isBoxSquarish(detection) || !hasValidAreaRatio(detection)) {
        continue;
      }
      
      if (detection.ta > largestArea) {
        largestArea = detection.ta;
        selectedAlgae = detection;
      }
    }
  }
  
  private double[] calculateControlValues(double targetX, double targetY) {
    // Map vision coordinates to robot control values
    double mappedTX = map(targetX, TX_MIN, TX_MAX, -1.0, 1.0);
    double mappedTY = map(targetY, TY_MIN, TY_MAX, 0.0, 1.0);
    
    // Calculate control values for driving
    double forwardVelocity = -mappedTY * kForwardP;
    double rotationVelocity = -mappedTX * kRotationP;
    
    // Log for debugging
    SmartDashboard.putNumber("Algae_MappedTX", mappedTX);
    SmartDashboard.putNumber("Algae_MappedTY", mappedTY);
    
    return new double[] {forwardVelocity, rotationVelocity};
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
      SmartDashboard.putNumber("Algae_LeftX", 0);
      SmartDashboard.putNumber("Algae_CenterX", 0);
      SmartDashboard.putNumber("Algae_TargetPointX", 0);
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
    selectedAlgae = null;
    // m_swerve.setControl(roboDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    // End only when parent commands end it
    return false;
  }
} 