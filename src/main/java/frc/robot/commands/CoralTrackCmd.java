// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GripperSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

/* Tracks coral objects using NetworkTables vision data and drives toward them */
public class CoralTrackCmd extends Command {
  // Selection algorithm enum
  public enum SelectionAlgorithm {
    LOWEST_FIRST,        // Select the lowest coral in frame (highest y value)
    LOWER_CENTER_CLOSEST, // Select coral closest to bottom center
    BIGGEST,             // Select largest coral by area
    THRESHOLD_RATIO_LOWEST, // Filter by aspect ratio, then select lowest
    WEIGHTED_Y_DISTANCE   // Weighted combination of aspect ratio and y position
  }
  
  // Detection class to store vision data
  private class CoralDetection {
    public int id;
    public double tx;  // x coordinate in normalized (-1 to 1) space
    public double ty;  // y coordinate in normalized (-1 to 1) space
    public double ta;  // area (0-100% of image)
    public double aspectRatio; // width / height
    public double[] corners; // [x1,y1,x2,y2,x3,y3,x4,y4]
    
    // Constructor
    public CoralDetection(int id, double tx, double ty, double ta, double[] corners) {
      this.id = id;
      this.tx = tx;
      this.ty = ty;
      this.ta = ta;
      this.corners = corners;
      
      // Calculate width and height from corners
      double[] dimensions = getBoundingBoxDimensions();
      double width = dimensions[0];
      double height = dimensions[1];
      this.aspectRatio = (height > 0) ? width / height : 0;
    }
    
    // Get bounding box dimensions [width, height, minX, maxX, minY, maxY]
    public double[] getBoundingBoxDimensions() {
      if (corners == null || corners.length < 8) {
        return new double[] {0, 0, 0, 0, 0, 0};
      }
      
      double minX = Double.MAX_VALUE;
      double maxX = -Double.MAX_VALUE;
      double minY = Double.MAX_VALUE;
      double maxY = -Double.MAX_VALUE;
      
      // Process all corner points
      for (int i = 0; i < 4; i++) {
        double x = corners[i * 2];
        double y = corners[i * 2 + 1];
        
        minX = Math.min(minX, x);
        maxX = Math.max(maxX, x);
        minY = Math.min(minY, y);
        maxY = Math.max(maxY, y);
      }
      
      return new double[] {maxX - minX, maxY - minY, minX, maxX, minY, maxY};
    }
  }
  
  // Constants
  // PID Constants
  private static final double kForwardP = 3.0;
  private static final double kRotationP = 1.8;
  
  // Aspect ratio for filtering coral objects
  private static final double MAX_ASPECT_RATIO = 1.8;
  private static final double MIN_ASPECT_RATIO = 0.5;
  
  // Mixed algorithm weights
  private static final double ASPECT_RATIO_WEIGHT = 0.7;
  private static final double Y_POSITION_WEIGHT = 0.3;
  
  // Tracking target point in bounding box using Cartesian coordinates (-1 to 1, -1 to 1)
  private static final double TARGET_OFFSET_X = -0.8;  // Default: 80% toward left edge from center
  private static final double TARGET_OFFSET_Y = 0.0;   // Default: vertical center
  
  // Subsystems and configuration
  private final CommandSwerveDrivetrain m_swerve;
  private final GripperSubsystem m_gripper;
  private final String networkTableName = "vision";
  private final JSONParser parser = new JSONParser();
  
  // Algorithm selection and filtering 
  private SelectionAlgorithm currentAlgorithm = SelectionAlgorithm.THRESHOLD_RATIO_LOWEST;
  private double targetAspectRatioMin = MIN_ASPECT_RATIO;
  private double targetAspectRatioMax = MAX_ASPECT_RATIO;
  
  // Weights for weighted algorithm
  private double aspectRatioWeight = ASPECT_RATIO_WEIGHT;
  private double yPositionWeight = Y_POSITION_WEIGHT;
  
  // Target point in normalized coordinates (-1 to 1, -1 to 1)
  private double targetOffsetX = TARGET_OFFSET_X;
  private double targetOffsetY = TARGET_OFFSET_Y;
  
  // Vision field limits
  private static final double TX_MIN = -1.0;
  private static final double TX_MAX = 1.0;
  private static final double TY_MIN = -1.0;
  private static final double TY_MAX = 1.0;
  
  // PID controllers
  private final PIDController rotationPID;
  private final PIDController forwardPID;
  
  // Drive configuration
  private final SwerveRequest.RobotCentric roboDrive;
  
  // Current tracking state
  private CoralDetection selectedCoral = null;
  
  public CoralTrackCmd(CommandSwerveDrivetrain swerve, GripperSubsystem gripper) {
    m_swerve = swerve;
    m_gripper = gripper;
    addRequirements(m_swerve, m_gripper);
    
    // Create PID controllers
    rotationPID = new PIDController(kRotationP, 0, 0);
    forwardPID = new PIDController(kForwardP, 0, 0);
    
    // Set input range for rotation PID (helps with wrap-around)
    rotationPID.enableContinuousInput(-1.0, 1.0);
    
    double maxSpeed = MetersPerSecond.of(1.5).in(MetersPerSecond);
    roboDrive = new SwerveRequest.RobotCentric()
      .withDeadband(maxSpeed * 0.035)
      .withDriveRequestType(DriveRequestType.Velocity);
  }

  @Override
  public void initialize() {
    // Initialize SmartDashboard values
    SmartDashboard.putNumber("Coral_ForwardVelocity", 0);
    SmartDashboard.putNumber("Coral_RotationVelocity", 0);
    SmartDashboard.putString("Coral_Algorithm", currentAlgorithm.toString());
    SmartDashboard.putNumber("Coral_TargetOffsetX", targetOffsetX);
    SmartDashboard.putNumber("Coral_TargetOffsetY", targetOffsetY);
    SmartDashboard.putNumber("Coral_AspectRatioMin", targetAspectRatioMin);
    SmartDashboard.putNumber("Coral_AspectRatioMax", targetAspectRatioMax);
    SmartDashboard.putNumber("Coral_AspectRatioWeight", aspectRatioWeight);
    SmartDashboard.putNumber("Coral_YPositionWeight", yPositionWeight);
    SmartDashboard.putNumber("Coral_AspectRatio", 0);
    SmartDashboard.putNumber("Coral_TargetX", 0);
    SmartDashboard.putNumber("Coral_TargetY", 0);
    SmartDashboard.putNumber("Coral_TargetPointX", 0);
    SmartDashboard.putNumber("Coral_TargetPointY", 0);
    SmartDashboard.putNumber("Coral_MappedTX", 0);
    SmartDashboard.putNumber("Coral_MappedTY", 0);
    SmartDashboard.putBoolean("Vision_HasTarget", false);
    SmartDashboard.putNumber("Vision_DetectionCount", 0);
    SmartDashboard.putNumber("Coral_Selected_ID", -1);
    SmartDashboard.putNumber("Coral_Selected_TX", 0);
    SmartDashboard.putNumber("Coral_Selected_TY", 0);
    SmartDashboard.putNumber("Coral_Selected_Area", 0);
    SmartDashboard.putNumber("Coral_Selected_AspectRatio", 0);
    SmartDashboard.putNumber("Coral_CenterX", 0);
    SmartDashboard.putNumber("Coral_CenterY", 0);
    
    // Reset tracking state
    selectedCoral = null;
    
    // Reset PID controllers
    rotationPID.reset();
    forwardPID.reset();
  }

  @Override
  public void execute() {
    // Process vision data and drive toward target if found
    processVisionData();
    
    if (selectedCoral != null) {
      // Calculate drive values
      double[] targetPoint = calculateTargetPoint(selectedCoral);
      
      // Use PID controllers
      double mappedTX = map(targetPoint[0], TX_MIN, TX_MAX, -1.0, 1.0);
      double mappedTY = map(targetPoint[1], TY_MIN, TY_MAX, 0.0, 1.0);
      
      double rotationVelocity = rotationPID.calculate(mappedTX, 0); // Target is center (0)
      double forwardVelocity = forwardPID.calculate(mappedTY, 0);   // Target is 0 (minimum distance)
      
      // Log control values
      SmartDashboard.putNumber("Coral_ForwardVelocity", forwardVelocity);
      SmartDashboard.putNumber("Coral_RotationVelocity", rotationVelocity);
      SmartDashboard.putNumber("Coral_MappedTX", mappedTX);
      SmartDashboard.putNumber("Coral_MappedTY", mappedTY);
      
      // Drive robot using calculated velocities
      m_swerve.setControl(
        roboDrive
          .withVelocityX(-forwardVelocity) // Forward velocity
          .withVelocityY(0.0)               // No side-to-side motion
          .withRotationalRate(rotationVelocity) // Rotation velocity
      );
    }
  }

  private void processVisionData() {
    // Get vision data from NetworkTables
    NetworkTable table = NetworkTableInstance.getDefault().getTable(networkTableName);
    NetworkTableEntry jsonEntry = table.getEntry("coral_detections");
    
    String jsonData = jsonEntry.getString("");
    boolean hasTarget = !jsonData.isEmpty();
    SmartDashboard.putBoolean("Vision_HasTarget", hasTarget);
    
    if (hasTarget) {
      try {
        // Parse JSON data
        JSONObject root = (JSONObject) parser.parse(jsonData);
        JSONArray detections = (JSONArray) root.get("detections");
        
        if (detections != null && !detections.isEmpty()) {
          int count = detections.size();
          SmartDashboard.putNumber("Vision_DetectionCount", count);
          
          // Convert JSON to CoralDetection objects
          CoralDetection[] coralDetections = new CoralDetection[count];
          for (int i = 0; i < count; i++) {
            JSONObject detection = (JSONObject) detections.get(i);
            int id = ((Long) detection.get("id")).intValue();
            double tx = (Double) detection.get("tx");
            double ty = (Double) detection.get("ty");
            double ta = (Double) detection.get("ta");
            
            // Get corners array
            JSONArray cornersJson = (JSONArray) detection.get("corners");
            double[] corners = new double[cornersJson.size()];
            for (int j = 0; j < cornersJson.size(); j++) {
              corners[j] = (Double) cornersJson.get(j);
            }
            
            coralDetections[i] = new CoralDetection(id, tx, ty, ta, corners);
          }
          
          // Select coral based on algorithm
          selectCoral(coralDetections);
        } else {
          SmartDashboard.putNumber("Vision_DetectionCount", 0);
          selectedCoral = null;
        }
      } catch (ParseException e) {
        SmartDashboard.putNumber("Vision_DetectionCount", 0);
        selectedCoral = null;
      }
    } else {
      SmartDashboard.putNumber("Vision_DetectionCount", 0);
      selectedCoral = null;
    }
    
    displaySelectedCoral();
  }
  
  private double[] calculateTargetPoint(CoralDetection detection) {
    // Calculate bounding box dimensions
    double[] dimensions = detection.getBoundingBoxDimensions();
    double width = dimensions[0];
    double height = dimensions[1];
    double minX = dimensions[2];
    double maxX = dimensions[3];
    double minY = dimensions[4];
    double maxY = dimensions[5];
    
    double centerX = (minX + maxX) / 2.0;
    double centerY = (minY + maxY) / 2.0;
    
    // For normalized coordinates, the target point is directly based on the target offset
    double targetX = detection.tx + targetOffsetX * width;
    double targetY = detection.ty + targetOffsetY * height;
    
    // Ensure target point is within bounds
    targetX = Math.max(TX_MIN, Math.min(TX_MAX, targetX));
    targetY = Math.max(TY_MIN, Math.min(TY_MAX, targetY));
    
    // Log for debugging
    SmartDashboard.putNumber("Coral_CenterX", centerX);
    SmartDashboard.putNumber("Coral_CenterY", centerY);
    SmartDashboard.putNumber("Coral_TargetX", targetX);
    SmartDashboard.putNumber("Coral_TargetY", targetY);
    SmartDashboard.putNumber("Coral_TargetPointX", targetX);
    SmartDashboard.putNumber("Coral_TargetPointY", targetY);
    SmartDashboard.putNumber("Coral_TargetOffsetX", targetOffsetX);
    SmartDashboard.putNumber("Coral_TargetOffsetY", targetOffsetY);
    
    return new double[] {targetX, targetY};
  }
  
  private boolean isAspectRatioValid(CoralDetection detection) {
    // Check if aspect ratio is within acceptable range
    double aspectRatio = detection.aspectRatio;
    SmartDashboard.putNumber("Coral_AspectRatio", aspectRatio);
    
    return aspectRatio >= targetAspectRatioMin && aspectRatio <= targetAspectRatioMax;
  }
  
  private void selectCoral(CoralDetection[] detections) {
    switch (currentAlgorithm) {
      case LOWEST_FIRST:
        selectLowestCoral(detections);
        break;
      case LOWER_CENTER_CLOSEST:
        selectClosestToBottomCenter(detections);
        break;
      case BIGGEST:
        selectBiggestCoral(detections);
        break;
      case THRESHOLD_RATIO_LOWEST:
        selectThresholdRatioLowest(detections);
        break;
      case WEIGHTED_Y_DISTANCE:
        selectWeightedYDistance(detections);
        break;
      default:
        selectThresholdRatioLowest(detections);
        break;
    }
  }
  
  private void selectLowestCoral(CoralDetection[] detections) {
    // Select the lowest coral in frame (highest ty value)
    selectedCoral = null;
    double highestTY = -Double.MAX_VALUE;
    
    for (CoralDetection detection : detections) {
      if (detection.ty > highestTY) {
        highestTY = detection.ty;
        selectedCoral = detection;
      }
    }
  }
  
  private void selectClosestToBottomCenter(CoralDetection[] detections) {
    // Select coral closest to bottom center
    selectedCoral = null;
    double closestDistance = Double.MAX_VALUE;
    
    for (CoralDetection detection : detections) {
      // Calculate distance to bottom center (0, 1)
      double distToBottomCenter = Math.sqrt(
          Math.pow(detection.tx, 2) + 
          Math.pow(detection.ty - 1.0, 2));
      
      if (distToBottomCenter < closestDistance) {
        closestDistance = distToBottomCenter;
        selectedCoral = detection;
      }
    }
  }
  
  private void selectBiggestCoral(CoralDetection[] detections) {
    // Select the largest coral (by area)
    selectedCoral = null;
    double largestArea = -Double.MAX_VALUE;
    
    for (CoralDetection detection : detections) {
      if (detection.ta > largestArea) {
        largestArea = detection.ta;
        selectedCoral = detection;
      }
    }
  }
  
  private void selectThresholdRatioLowest(CoralDetection[] detections) {
    // First filter by aspect ratio, then select the lowest
    // NOTE: With this algorithm, we may ignore closer objects if they don't match our aspect ratio criteria
    // This is mentioned in the comments at the request of the user
    selectedCoral = null;
    double highestTY = -Double.MAX_VALUE;
    
    for (CoralDetection detection : detections) {
      if (!isAspectRatioValid(detection)) {
        continue; // Skip detections with invalid aspect ratio
      }
      
      if (detection.ty > highestTY) {
        highestTY = detection.ty;
        selectedCoral = detection;
      }
    }
  }
  
  private void selectWeightedYDistance(CoralDetection[] detections) {
    // Weight both aspect ratio and y-position to select a target
    // NOTE: Similarly to the threshold approach, this may have limitations when objects are in our way
    // This is mentioned in the comments at the request of the user
    selectedCoral = null;
    double bestScore = -Double.MAX_VALUE;
    
    for (CoralDetection detection : detections) {
      // Calculate aspect ratio score (1.0 if perfect, less if not)
      double idealAspectRatio = (targetAspectRatioMax + targetAspectRatioMin) / 2.0;
      double aspectRatioRange = targetAspectRatioMax - targetAspectRatioMin;
      double aspectRatioScore = 1.0 - (Math.abs(detection.aspectRatio - idealAspectRatio) / (aspectRatioRange / 2.0));
      aspectRatioScore = Math.max(0.0, Math.min(1.0, aspectRatioScore));
      
      // Calculate y-position score (1.0 if at bottom of image, 0.0 if at top)
      double yPositionScore = (detection.ty + 1.0) / 2.0; // Map from [-1,1] to [0,1]
      
      // Calculate weighted score
      double score = (aspectRatioScore * aspectRatioWeight) + (yPositionScore * yPositionWeight);
      
      if (score > bestScore) {
        bestScore = score;
        selectedCoral = detection;
      }
    }
  }
  
  private double map(double value, double inMin, double inMax, double outMin, double outMax) {
    // Map a value from one range to another
    value = Math.max(inMin, Math.min(inMax, value));
    return outMin + (value - inMin) * (outMax - outMin) / (inMax - inMin);
  }
  
  private void displaySelectedCoral() {
    // Update dashboard with selected coral information
    if (selectedCoral != null) {
      SmartDashboard.putString("Coral_SelectionAlgorithm", currentAlgorithm.toString());
      SmartDashboard.putNumber("Coral_Selected_ID", selectedCoral.id);
      SmartDashboard.putNumber("Coral_Selected_TX", selectedCoral.tx);
      SmartDashboard.putNumber("Coral_Selected_TY", selectedCoral.ty);
      SmartDashboard.putNumber("Coral_Selected_Area", selectedCoral.ta);
      SmartDashboard.putNumber("Coral_Selected_AspectRatio", selectedCoral.aspectRatio);
    } else {
      SmartDashboard.putString("Coral_SelectionAlgorithm", "None");
      SmartDashboard.putNumber("Coral_Selected_ID", -1);
      SmartDashboard.putNumber("Coral_Selected_TX", 0);
      SmartDashboard.putNumber("Coral_Selected_TY", 0);
      SmartDashboard.putNumber("Coral_Selected_Area", 0);
      SmartDashboard.putNumber("Coral_Selected_AspectRatio", 0);
      SmartDashboard.putNumber("Coral_MappedTX", 0);
      SmartDashboard.putNumber("Coral_MappedTY", 0);
      SmartDashboard.putNumber("Coral_RotationVelocity", 0);
      SmartDashboard.putNumber("Coral_ForwardVelocity", 0);
      SmartDashboard.putNumber("Coral_CenterX", 0);
      SmartDashboard.putNumber("Coral_CenterY", 0);
      SmartDashboard.putNumber("Coral_TargetPointX", 0);
      SmartDashboard.putNumber("Coral_TargetPointY", 0);
    }
  }
  
  // Public methods to configure algorithm and settings
  public void setSelectionAlgorithm(SelectionAlgorithm algorithm) {
    currentAlgorithm = algorithm;
  }
  
  public void setAspectRatioRange(double min, double max) {
    targetAspectRatioMin = min;
    targetAspectRatioMax = max;
  }
  
  public void setWeights(double aspectRatioWeight, double yPositionWeight) {
    this.aspectRatioWeight = aspectRatioWeight;
    this.yPositionWeight = yPositionWeight;
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
    selectedCoral = null;
    m_swerve.setControl(
      roboDrive
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0)
    );
  }

  @Override
  public boolean isFinished() {
    // End when gripper has coral
    return m_gripper.hasCoral();
  }
} 