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
import frc.robot.Constants.Elevator;

import static edu.wpi.first.units.Units.*;

public class ElevatorSubsystem extends SubsystemBase {
  
  private final TalonFX m_motor       = new TalonFX(Elevator.kMotor1Port);
  private final TalonFX m_slaveMotor = new TalonFX(Elevator.kMotor2Port);

  private final TalonFXConfiguration m_talonConfig = new TalonFXConfiguration();
  private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0);
  private final MotionMagicVelocityVoltage m_motionMagicVel = new MotionMagicVelocityVoltage(0).withSlot(1);

  private final NeutralOut m_neutral = new NeutralOut();
  private final StaticBrake m_brake = new StaticBrake();

  private double elevatorHeight = 0;

  private boolean isGoalReached = false;
  private boolean isMotorsSet = false;

  private final SendableChooser<Integer> m_offsetChooser = new SendableChooser<>();
  private double eleOffset = 0;


  /** Creates a new ElevatorSubsytem. */
  public ElevatorSubsystem() {
    m_talonConfig.Slot0.kP = Elevator.kElevatorKp; // An error of 1 rotation results in 2.4 V output
    m_talonConfig.Slot0.kI = Elevator.kElevatorKi; // No output for integrated error
    m_talonConfig.Slot0.kD = Elevator.kElevatorKd; // A velocity of 1 rps results in 0.1 V output
    m_talonConfig.Slot0.withGravityType(GravityTypeValue.Elevator_Static)
    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
    .kG = Elevator.kElevatorkG;
    m_talonConfig.Voltage.withPeakForwardVoltage(Volts.of(Elevator.kVoltageLimit))
    .withPeakReverseVoltage(Volts.of(-Elevator.kVoltageLimit));
    m_talonConfig.CurrentLimits.withSupplyCurrentLimit(Elevator.kAmpLimit);

    m_talonConfig.MotionMagic.MotionMagicCruiseVelocity = Elevator.kElevatorMMCV;
    m_talonConfig.MotionMagic.MotionMagicAcceleration = Elevator.kElevatorMMA;
    m_talonConfig.MotionMagic.MotionMagicJerk = Elevator.kElevatorMMJ;

    m_talonConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    m_talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply configs
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_motor.getConfigurator().apply(m_talonConfig);
      status = m_slaveMotor.getConfigurator().apply(m_talonConfig);

      if (status.isOK()) break;
          
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
    m_motor.setPosition(0);
    m_slaveMotor.setPosition(0);

    m_slaveMotor.setControl(new Follower(m_motor.getDeviceID(), false));

    m_offsetChooser.setDefaultOption("0", 0);
    m_offsetChooser.addOption("1", 1);
    m_offsetChooser.addOption("2", 2);
    m_offsetChooser.addOption("3", 3);
    m_offsetChooser.addOption("4", 4);
    m_offsetChooser.addOption("5", 5);
    m_offsetChooser.addOption("6", 6);
    m_offsetChooser.addOption("7", 7);
    m_offsetChooser.addOption("8", 8);
    m_offsetChooser.addOption("9", 9);
    m_offsetChooser.addOption("10", 10);
    m_offsetChooser.addOption("-1", -1);
    m_offsetChooser.addOption("-2", -2);
    m_offsetChooser.addOption("-3", -3);
    m_offsetChooser.addOption("-4", -4);
    m_offsetChooser.addOption("-5", -5);
    m_offsetChooser.addOption("-6", -6);
    m_offsetChooser.addOption("-7", -7);
    m_offsetChooser.addOption("-8", -8);
    m_offsetChooser.addOption("-9", -9);
    m_offsetChooser.addOption("-10", -10);
    m_offsetChooser.onChange(this::offsetListener);
    SmartDashboard.putData("EleOffsetChooser", m_offsetChooser);
    SmartDashboard.setPersistent("EleOffsetChooser");
    }
    
    public void offsetListener(double selected)
  {
    if (m_offsetChooser != null) {
      eleOffset = selected*0.02; 
    } else eleOffset = 0;
    if (eleOffset > 0.2) eleOffset = 0.2;
    else if (eleOffset < -0.2) eleOffset = -0.2;
    else eleOffset = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorHeight = getEncoderDistance();
    SmartDashboard.putNumber("Elevator/ElevatorHeight", elevatorHeight);
    SmartDashboard.putBoolean("Elevator/EleGoalReached", isGoalReached);
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
    
    if(goal < Elevator.kMinElevatorHeightMeters) goal = Elevator.kMinElevatorHeightMeters;
    else if(goal > Elevator.kMaxElevatorHeightMeters) goal = Elevator.kMaxElevatorHeightMeters;
    //if(getElevatorHeight() > 0.6 && goal < 0.6) goal = 0.5;

    goal += eleOffset;
    m_motor.setControl(m_motionMagic.withPosition(goal / (Elevator.kElevatorDrumRadius * 2 * Math.PI / Elevator.kElevatorGearing)));
    isGoalReached = (Math.abs(getElevatorHeight() - goal) < Elevator.kElevatorTolerance);
  }

  /** Stop the control loop and motor output. */
  public void stop() {
    m_motor.setControl(m_brake);
  }

  public double getEncoderDistance() { //Linear Distance
    return m_motor.getPosition().getValueAsDouble() * (Elevator.kElevatorDrumRadius * 2 * Math.PI / Elevator.kElevatorGearing);
  }

  public double getElevatorHeight() 
  {
    return elevatorHeight;
  }
  
  public Command reachGoalCommand(double h)
  {
    return run(() -> {
      reachGoal(h);
    }).until(() -> { 
      return Math.abs(elevatorHeight - h) < Elevator.kElevatorTolerance; 
    });
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
    if(!isMotorsSet) isMotorsSet = (m_motor.getPosition().getValueAsDouble() == m_slaveMotor.getPosition().getValueAsDouble() 
                                                                             && m_motor.getPosition().getValueAsDouble() == 0);
    return isMotorsSet;
  }

    public void speedControl(double v)
  {
    //m_motor.setControl(m_motionMagicVel.withVelocity(v*Arm.SecondJoint.kArmReduction));
    m_motor.set(v);
  }

  public Command eleSpeedControl(double v)
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

  public Command beReady()
  {
    return run(() -> {if(elevatorHeight < 0.05) reachGoal(0.05);});
  }


}
