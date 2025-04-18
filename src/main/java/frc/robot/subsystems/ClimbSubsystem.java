// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Climb;

import static edu.wpi.first.units.Units.*;

public class ClimbSubsystem extends SubsystemBase {
  
  private final TalonFX m_motor       = new TalonFX(Climb.kMotor1Port);

  private final TalonFXConfiguration m_talonConfig = new TalonFXConfiguration();
  private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0);

  private final StaticBrake m_brake = new StaticBrake();
  private final NeutralOut m_neutral = new NeutralOut();


  private double climbPos = 0;

  private boolean isGoalReached = false;
  private boolean isMotorsSet = false;



  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
  m_talonConfig.Slot0.kP = Climb.kClimbKp; // An error of 1 rotation results in 2.4 V output
  m_talonConfig.Slot0.kI = Climb.kClimbKi; // No output for integrated error
  m_talonConfig.Slot0.kD = Climb.kClimbKd; // A velocity of 1 rps results in 0.1 V output
  m_talonConfig.Slot0.withGravityType(GravityTypeValue.Elevator_Static)
  .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
  .kG = Climb.kClimbkG;
  m_talonConfig.Voltage.withPeakForwardVoltage(Volts.of(Climb.kVoltageLimit))
  .withPeakReverseVoltage(Volts.of(-Climb.kVoltageLimit));
  m_talonConfig.CurrentLimits.withSupplyCurrentLimit(Climb.kAmpLimit);

  m_talonConfig.MotionMagic.MotionMagicCruiseVelocity = Climb.kClimbMMCV;
  m_talonConfig.MotionMagic.MotionMagicAcceleration = Climb.kClimbMMA;
  m_talonConfig.MotionMagic.MotionMagicJerk = Climb.kClimbMMJ;

  m_talonConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

  m_talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

  // Apply configs
  StatusCode status = StatusCode.StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status = m_motor.getConfigurator().apply(m_talonConfig);

    if (status.isOK()) break;
      
  }
  if (!status.isOK()) {
    System.out.println("Could not apply configs, error code: " + status.toString());
  }
  m_motor.setPosition(0);
  
  }
  

  @Override
  public void periodic() {
  // This method will be called once per scheduler run
  climbPos = getPosition();
  SmartDashboard.putNumber("Climb/ClimbPosition", climbPos);
  SmartDashboard.putBoolean("Climb/ClimbGoalReached", isGoalReached);
  if (RobotState.isTest()) m_motor.setControl(m_neutral);
  else if (RobotState.isDisabled()) m_motor.setControl(m_brake);

  if(!isMotorsSet)
  {
  if(m_motor.getPosition().getValueAsDouble() != 0)
  {
    resetMotorPosition();
  }
  else isMotorsSet = true;
  }

  }

  public void reachGoal(double goal) {
    //if(getClimbHeight() > 0.6 && goal < 0.6) goal = 0.5;
  m_motor.setControl(m_motionMagic.withPosition(goal / 360 * Climb.kClimbReduction));
  isGoalReached = (Math.abs(climbPos - goal) < Climb.kClimbTolerance);
  }

  /** Stop the control loop and motor output. */
  public void stop() {
  m_motor.setControl(m_brake);
  }

  public double getPosition() { //Linear Distance
  return m_motor.getPosition().getValueAsDouble() / Climb.kClimbReduction * 360;
  }
  
  public Command reachGoalCommand(double h)
  {
  return run(() -> {
    reachGoal(h);
  }).until(() -> { 
    return Math.abs(climbPos - h) < Climb.kClimbTolerance; 
  });
  }

  public Command closedCommand()
  {
    return reachGoalCommand(2);
  }

  public Command enterBarge()
  {
    return reachGoalCommand(140);
  }

  public Command eatBarge()
  {
    return reachGoalCommand(240);
  }

  public void resetMotorPosition()
  {
  m_motor.setPosition(0);
  }

  public boolean isGoalReached(){
  return isGoalReached;
  }

  public boolean checkMotorsSet()
  {
  if(!isMotorsSet) isMotorsSet = (m_motor.getPosition().getValueAsDouble() == 0);
  return isMotorsSet;
  }

  public void speedControl(double v)
  {
  //m_motor.setControl(m_motionMagicVel.withVelocity(v*Arm.SecondJoint.kArmReduction));
  m_motor.set(v);
  }

  public Command climbSpeedControl(double v)
  {
  return run(() -> speedControl(v)).finallyDo(this::stop);
  }

  public Command resetMotorPositionCmd()
  {
  return runOnce(() -> resetMotorPosition());
  }

  public Command resetEncoderCmd()
  {
  return runOnce(() -> m_motor.setPosition(0));
  }




}
