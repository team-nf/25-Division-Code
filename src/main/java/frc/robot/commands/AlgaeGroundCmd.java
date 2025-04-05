// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.MainMechStateMachine;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeGroundCmd extends Command {
  /** Creates a new CoralIntake. */

  private MainMechStateMachine m_mainMech;
  private CommandSwerveDrivetrain m_swerve;
  private final String limelightName = "limelight-obj";
  
  // Constants for mapping
  private static final double TX_MIN = -29.8;
  private static final double TX_MAX = 29.8;
  private static final double TY_MIN = -24.85;
  private static final double TY_MAX = 24.85;
  
  // PID Constants for algae targeting
  private static final double kForwardP = 0.05;
  private static final double kRotationP = 0.03;
  
  // Selected algae tracking
  private RawDetection selectedAlgae = null;
  private double rotationVelocity = 0.0;
  private double forwardVelocity = 0.0;
  private double mappedTX = 0.0;
  private double mappedTY = 0.0;
  private double targetPointX = 0.0; // X coordinate of the target point on algae

  public AlgaeGroundCmd(MainMechStateMachine mainMech, CommandSwerveDrivetrain swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_mainMech = mainMech;
    m_swerve = swerve;
    addRequirements(m_mainMech.getArmSubsystem(), m_mainMech.getElevatorSubsystem());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Ensure we're using the appropriate pipeline for neural detection
    // You may need to adjust the pipeline index to match your Limelight configuration
    LimelightHelpers.setPipelineIndex(limelightName, 0);
    
    // Reset selected algae and control values
    selectedAlgae = null;
    rotationVelocity = 0.0;
    forwardVelocity = 0.0;
    mappedTX = 0.0;
    mappedTY = 0.0;
    targetPointX = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_mainMech.MechStateControl("AlgaeGround");
    
    // Get and process Limelight detections
    processLimelightDetections();
  }

  private void processLimelightDetections() {
    // Check if we have a valid target
    boolean hasTarget = LimelightHelpers.getTV(limelightName);
    SmartDashboard.putBoolean("LL_HasTarget", hasTarget);
    
    if (hasTarget) {
      // Get raw neural detections
      RawDetection[] detections = LimelightHelpers.getRawDetections(limelightName);
      
      // Display basic target info
      SmartDashboard.putNumber("LL_TX", LimelightHelpers.getTX(limelightName));
      SmartDashboard.putNumber("LL_TY", LimelightHelpers.getTY(limelightName));
      SmartDashboard.putNumber("LL_TA", LimelightHelpers.getTA(limelightName));
      
      // Display neural detector class information
      SmartDashboard.putString("LL_DetectorClass", LimelightHelpers.getDetectorClass(limelightName));
      SmartDashboard.putNumber("LL_DetectorClassIndex", LimelightHelpers.getDetectorClassIndex(limelightName));
      
      // Select algae based on different algorithms
      if (detections != null && detections.length > 0) {
        SmartDashboard.putNumber("LL_DetectionCount", detections.length);
        
        // Apply selection algorithms
        selectLowestAlgae(detections);
        // Uncomment to use bottom-center distance selection instead
        // selectClosestToBottomCenter(detections);
        
        // Map values and calculate control speeds
        if (selectedAlgae != null) {
          calculateTargetPoint();
          calculateControlValues();
        }
      } else {
        SmartDashboard.putNumber("LL_DetectionCount", 0);
        resetSelection();
      }
    } else {
      SmartDashboard.putNumber("LL_DetectionCount", 0);
      resetSelection();
    }
  }
  
  /**
   * Calculates the target point on the algae (20% from left edge to center)
   */
  private void calculateTargetPoint() {
    // Calculate the leftmost and rightmost x coordinates
    double leftX = Math.min(
      Math.min(selectedAlgae.corner0_X, selectedAlgae.corner1_X),
      Math.min(selectedAlgae.corner2_X, selectedAlgae.corner3_X)
    );
    
    double rightX = Math.max(
      Math.max(selectedAlgae.corner0_X, selectedAlgae.corner1_X),
      Math.max(selectedAlgae.corner2_X, selectedAlgae.corner3_X)
    );
    
    // Calculate center X
    double centerX = (leftX + rightX) / 2.0;
    
    // Calculate target point X (20% from left edge to center - close to left side)
    double targetX = leftX + (centerX - leftX) * 0.2;
    
    // Get Limelight resolution from JSON data
    double resolutionX = 320.0; // Default fallback resolution
    LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
    if (results != null && results.error == null) {
      // Try to extract resolution from JSON
      String jsonData = LimelightHelpers.getJSONDump(limelightName);
      if (jsonData != null && !jsonData.isEmpty()) {
        // Look for width value in JSON data
        int widthIndex = jsonData.indexOf("\"width\":");
        if (widthIndex >= 0) {
          try {
            String widthStr = jsonData.substring(widthIndex + 8, jsonData.indexOf(",", widthIndex));
            resolutionX = Double.parseDouble(widthStr);
            SmartDashboard.putNumber("Limelight_ResolutionX", resolutionX);
          } catch (Exception e) {
            // Use default resolution if parsing fails
          }
        }
      }
    }
    
    // Calculate the angle to this point from camera center
    // First convert from pixel coordinates to normalized coordinates
    double normalizedX = (targetX / resolutionX) - 0.5;
    
    // Then convert to angular coordinates (approximation)
    targetPointX = normalizedX * (TX_MAX - TX_MIN);
    
    // Display values for debugging
    SmartDashboard.putNumber("Algae_LeftX", leftX);
    SmartDashboard.putNumber("Algae_CenterX", centerX);
    SmartDashboard.putNumber("Algae_TargetPointX", targetPointX);
  }
  
  /**
   * Selects the lowest algae in the frame (highest ty value)
   */
  private void selectLowestAlgae(RawDetection[] detections) {
    selectedAlgae = null;
    double highestTY = -Double.MAX_VALUE;
    
    for (RawDetection detection : detections) {
      if (detection.tync > highestTY) {
        highestTY = detection.tync;
        selectedAlgae = detection;
      }
    }
    
    displaySelectedAlgae("Lowest");
  }
  
  /**
   * Selects the algae closest to the bottom center of the frame
   * using distance formula sqrt((tx+30)^2 + ty^2)
   */
  private void selectClosestToBottomCenter(RawDetection[] detections) {
    selectedAlgae = null;
    double closestDistance = Double.MAX_VALUE;
    
    for (RawDetection detection : detections) {
      // Calculate distance to bottom center (approximately (0, TY_MIN))
      // Note: Adding TX_MAX to tx would place it at the rightmost edge, not center
      // For bottom center, we want the absolute distance from x=0
      double distToBottomCenter = Math.sqrt(
          Math.pow(detection.txnc, 2) + 
          Math.pow(detection.tync - TY_MIN, 2));
      
      if (distToBottomCenter < closestDistance) {
        closestDistance = distToBottomCenter;
        selectedAlgae = detection;
      }
    }
    
    displaySelectedAlgae("BottomCenter");
  }
  
  /**
   * Maps tx and ty values to (-1, 1) range and calculates control values
   */
  private void calculateControlValues() {
    // Map targetPointX from (TX_MIN, TX_MAX) to (-1, 1) for rotation
    mappedTX = map(targetPointX, TX_MIN, TX_MAX, -1.0, 1.0);
    
    // Map TY from (TY_MIN, TY_MAX) to (-1, 1) for forward motion
    mappedTY = map(selectedAlgae.tync, TY_MIN, TY_MAX, -1.0, 1.0);
    
    // Calculate control values
    rotationVelocity = -mappedTX * kRotationP; // Negative because positive tx means target is to the right
    forwardVelocity = -mappedTY * kForwardP;   // Adjust sign as needed based on your robot's coordinate system
    
    // Display mapped and control values
    SmartDashboard.putNumber("Algae_MappedTX", mappedTX);
    SmartDashboard.putNumber("Algae_MappedTY", mappedTY);
    SmartDashboard.putNumber("Algae_RotationVelocity", rotationVelocity);
    SmartDashboard.putNumber("Algae_ForwardVelocity", forwardVelocity);
  }
  
  /**
   * Helper method to map a value from one range to another
   */
  private double map(double value, double inMin, double inMax, double outMin, double outMax) {
    // Clamp the input value to the input range
    value = Math.max(inMin, Math.min(inMax, value));
    
    // Map the clamped value to the output range
    return outMin + (value - inMin) * (outMax - outMin) / (inMax - inMin);
  }
  
  /**
   * Displays information about the selected algae
   */
  private void displaySelectedAlgae(String algorithm) {
    if (selectedAlgae != null) {
      SmartDashboard.putString("Algae_SelectionAlgorithm", algorithm);
      SmartDashboard.putNumber("Algae_Selected_ClassID", selectedAlgae.classId);
      SmartDashboard.putNumber("Algae_Selected_TX", selectedAlgae.txnc);
      SmartDashboard.putNumber("Algae_Selected_TY", selectedAlgae.tync);
      SmartDashboard.putNumber("Algae_Selected_Area", selectedAlgae.ta);
    } else {
      resetSelection();
    }
  }
  
  /**
   * Resets all selection and control values
   */
  private void resetSelection() {
    selectedAlgae = null;
    rotationVelocity = 0.0;
    forwardVelocity = 0.0;
    mappedTX = 0.0;
    mappedTY = 0.0;
    targetPointX = 0.0;
    
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    resetSelection();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_mainMech.isGoalReached();
  }
}
