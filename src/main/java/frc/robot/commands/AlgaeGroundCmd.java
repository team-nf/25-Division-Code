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
  private final String limelightName = "limelight-obj"; // Default Limelight name (empty string for "limelight")

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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_mainMech.MechStateControl("AlgaeGround");
    
    // Get and print neural detector information
    printLimelightNeuralDetection();
  }

  private void printLimelightNeuralDetection() {
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
      
      // Print information about each detected object
      if (detections != null && detections.length > 0) {
        SmartDashboard.putNumber("LL_DetectionCount", detections.length);
        
        // Print details of the first detection (most confident)
        if (detections.length > 0) {
          RawDetection primaryDetection = detections[0];
          SmartDashboard.putNumber("LL_Detection_ClassID", primaryDetection.classId);
          SmartDashboard.putNumber("LL_Detection_TX", primaryDetection.txnc);
          SmartDashboard.putNumber("LL_Detection_TY", primaryDetection.tync);
          SmartDashboard.putNumber("LL_Detection_Area", primaryDetection.ta);
        }
      } else {
        SmartDashboard.putNumber("LL_DetectionCount", 0);
      }
    } else {
      // Clear values if no target
      SmartDashboard.putNumber("LL_DetectionCount", 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_mainMech.isGoalReached();
  }
}
