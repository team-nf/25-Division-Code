// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.MetersPerSecond;

/* 
 * Receives error values from a co-processor via NetworkTables,
 * processes them through PID controllers, and applies the result to the drivetrain.
 */
public class AlgaeCustomTrackCmd extends Command {
  // Subsystem and network tables configuration
  private CommandSwerveDrivetrain m_swerve;
  private final String NT_TABLE_NAME = "AlgaeTracker";
  
  // Network table entries
  private NetworkTable m_table;
  private NetworkTableEntry m_hasTarget;       // Whether a target is detected (boolean)
  private NetworkTableEntry m_rotationPower;   // Rotation error to apply [-1.0, 1.0]
  private NetworkTableEntry m_forwardPower;    // Forward error to apply [-1.0, 1.0]
  
  // PID configuration
  private static final double kForwardP = 3.0;
  private static final double kRotationP = 1.8;
  
  // PID controllers
  private final PIDController rotationPID;
  private final PIDController forwardPID;
  
  // Filters for smoothing
  private LinearFilter velocityFilter = LinearFilter.movingAverage(5);
  private LinearFilter rotationFilter = LinearFilter.movingAverage(3);
  
  // Drive configuration
  private final SwerveRequest.RobotCentric roboDrive;

  public AlgaeCustomTrackCmd(CommandSwerveDrivetrain swerve) {
    m_swerve = swerve;
    addRequirements(m_swerve);
    
    // Set up NetworkTables
    m_table = NetworkTableInstance.getDefault().getTable(NT_TABLE_NAME);
    m_hasTarget = m_table.getEntry("hasTarget");
    m_rotationPower = m_table.getEntry("rotationPower");
    m_forwardPower = m_table.getEntry("forwardPower");
    
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
    // Reset PID controllers
    rotationPID.reset();
    forwardPID.reset();
    
    // Publish robot info and PID values to NetworkTables for co-processor
    m_table.getEntry("robotMaxSpeed").setDouble(m_swerve.getMaxSpeed());
    m_table.getEntry("robotMaxAngularRate").setDouble(m_swerve.getMaxAngularRate());
    m_table.getEntry("rotationP").setDouble(kRotationP);
    m_table.getEntry("forwardP").setDouble(kForwardP);
  }

  @Override
  public void execute() {
    // Get data from NetworkTables
    boolean hasTarget = m_hasTarget.getBoolean(false);
    SmartDashboard.putBoolean("AlgaeCustom_HasTarget", hasTarget);
    
    if (hasTarget) {
      // Get error values from NetworkTables
      double rotationError = m_rotationPower.getDouble(0.0);
      double forwardError = m_forwardPower.getDouble(0.0);
      
      // Calculate control outputs using PID
      double rotationPower = -rotationPID.calculate(rotationError, 0); // Target is 0 (no error)
      double forwardPower = -forwardPID.calculate(forwardError, 0);    // Target is 0 (no error)
      
      // Apply filtering for smoother motion
      forwardPower = velocityFilter.calculate(forwardPower);
      rotationPower = rotationFilter.calculate(rotationPower);
      
      // Log values
      SmartDashboard.putNumber("AlgaeCustom_RotationError", rotationError);
      SmartDashboard.putNumber("AlgaeCustom_ForwardError", forwardError);
      SmartDashboard.putNumber("AlgaeCustom_ForwardPower", forwardPower);
      SmartDashboard.putNumber("AlgaeCustom_RotationPower", rotationPower);
      
      // Drive robot using PID-calculated power values
      m_swerve.setControl(
        roboDrive
          .withVelocityX(forwardPower * m_swerve.getMaxSpeed()) // Scale by max speed
          .withVelocityY(0.0)                                   // No side-to-side motion
          .withRotationalRate(rotationPower * m_swerve.getMaxAngularRate()) // Scale by max angular rate
      );
    } else {
      // No valid target, stop robot
      m_swerve.setControl(
        roboDrive
          .withVelocityX(0)
          .withVelocityY(0)
          .withRotationalRate(0)
      );
      
      // Reset PID controllers when no target is visible
      rotationPID.reset();
      forwardPID.reset();
    }
  }
  
  // Methods to configure PID values
  public void setRotationPID(double p, double i, double d) {
    rotationPID.setPID(p, i, d);
    m_table.getEntry("rotationP").setDouble(p);
    m_table.getEntry("rotationI").setDouble(i);
    m_table.getEntry("rotationD").setDouble(d);
  }
  
  public void setForwardPID(double p, double i, double d) {
    forwardPID.setPID(p, i, d);
    m_table.getEntry("forwardP").setDouble(p);
    m_table.getEntry("forwardI").setDouble(i);
    m_table.getEntry("forwardD").setDouble(d);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain when command ends
    m_swerve.setControl(roboDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    // End only when parent commands end it
    return false;
  }
} 