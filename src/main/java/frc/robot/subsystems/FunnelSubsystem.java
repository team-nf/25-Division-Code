// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FunnelConstants;

public class FunnelSubsystem extends SubsystemBase {
  /** Creates a new FunnelSubsystem. */

  private final DigitalInput m_leftSensor = new DigitalInput(FunnelConstants.kLeftSensor);
  private final DigitalInput m_rightSensor = new DigitalInput(FunnelConstants.kRightSensor);
  
  public FunnelSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("FunnelLeftSensor", leftValue());
    SmartDashboard.putBoolean("FunnelRightSensor", rightValue());

  }

  public boolean leftValue()
  {
    return !m_leftSensor.get();
  }

  public boolean rightValue()
  {
    return !m_rightSensor.get();
  }

  public boolean isReady()
  {
    return leftValue() || rightValue();
  }
}
