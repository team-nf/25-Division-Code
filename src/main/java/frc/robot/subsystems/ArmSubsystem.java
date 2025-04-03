// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Arm;
import frc.robot.custom.ArmHalfEncoder;
import frc.robot.custom.ArmHalfEncoderSim;

public class ArmSubsystem extends SubsystemBase {

  private final TalonFX m_armFirstJointMotor = new TalonFX(Arm.FirstJoint.kMotorPort);
  private final TalonFX m_armSecondJointMotor = new TalonFX(Arm.SecondJoint.kMotorPort);

  private final CoastOut m_firstJointNeutralOut = new CoastOut();
  private final CoastOut m_secondJointNeutralOut = new CoastOut();
  private final StaticBrake m_firstJointBrake = new StaticBrake();
  private final StaticBrake m_secondJointBrake = new StaticBrake();


  private final MotionMagicVoltage m_firstJointMotionMagic = new MotionMagicVoltage(0).withSlot(0);
  private final MotionMagicVoltage m_secondJointMotionMagic = new MotionMagicVoltage(0).withSlot(0);

  private final ArmHalfEncoder m_firstJointHalfcoder = new ArmHalfEncoder(Arm.FirstJoint.kEncoderChannel, false, false);
  private final ArmHalfEncoder m_secondJointHalfcoder = new ArmHalfEncoder(Arm.SecondJoint.kEncoderChannel, true, true);

  private final DCMotor armFirstJointDC = DCMotor.getKrakenX60(1);
  private final DCMotor armSecondJointDC = DCMotor.getKrakenX60(1);

  /* 
  private final SingleJointedArmSim m_armSimJ1 =
      new SingleJointedArmSim(
          armFirstJointDC,
          Arm.FirstJoint.kArmReduction,
          SingleJointedArmSim.estimateMOI(Arm.FirstJoint.kArmLength, Arm.FirstJoint.kArmMass),
          Arm.FirstJoint.kArmLength,
          Arm.FirstJoint.kMinAngleRads,
          Arm.FirstJoint.kMaxAngleRads,
          true,
          0,
          Arm.FirstJoint.kArmEncoderDistPerPulse,
          0.0 // Add noise with a std-dev of 1 tick
      );

  private final SingleJointedArmSim m_armSimJ2 =
      new SingleJointedArmSim(
          armSecondJointDC,
          Arm.SecondJoint.kArmReduction,
          SingleJointedArmSim.estimateMOI(Arm.SecondJoint.kArmLength, Arm.SecondJoint.kArmMass),
          Arm.SecondJoint.kArmLength,
          Arm.SecondJoint.kMinAngleRads,
          Arm.SecondJoint.kMaxAngleRads,
          true,
          0,
          Arm.SecondJoint.kArmEncoderDistPerPulse,
          0.0 // Add noise with a std-dev of 1 tick
      );

  private final ArmHalfEncoderSim m_firstJointHalfcoderSim = new ArmHalfEncoderSim(m_firstJointHalfcoder);
  private final ArmHalfEncoderSim m_secondJointHalfcoderSim = new ArmHalfEncoderSim(m_secondJointHalfcoder);

  private final TalonFXSimState m_armFirstJointMotorSim;
  private final TalonFXSimState m_armSecondJointMotorSim;
  */

  private final SendableChooser<Integer> m_offsetChooser = new SendableChooser<>();

  private double firstJointAngle = 0;
  private double secondJointAngle = 0;

  private double armJ1limitCCW = Arm.FirstJoint.kMaxAngle;
  private double armJ1limitCW = Arm.FirstJoint.kMinAngle;
  private double armJ2limitCCW = Arm.SecondJoint.kMaxAngle;
  private double armJ2limitCW = Arm.SecondJoint.kMinAngle;
  
  private boolean isArmReady = false;
  private double armJ1LastAngle = -1;
  private double armJ2LastAngle = -1;

  private double armJ1MotorPos = 0;
  private double armJ2MotorPos = 0;

  private boolean isJ1GoalReached = false;
  private boolean isJ2GoalReached = false;

  private boolean isInitialReady = false;

  private boolean isMotorsSet = false;

  private double j2Offset = 0;

  public ArmSubsystem() {
        TalonFXConfiguration firstJointConfigs = new TalonFXConfiguration();
        firstJointConfigs.Slot0.kS = Arm.FirstJoint.kArmJoint1_kS;
        firstJointConfigs.Slot0.kV = Arm.FirstJoint.kArmJoint1_kV;
        firstJointConfigs.Slot0.kP = Arm.FirstJoint.kArmJoint1_kP;
        firstJointConfigs.Slot0.kI = Arm.FirstJoint.kArmJoint1_kI;
        firstJointConfigs.Slot0.kD = Arm.FirstJoint.kArmJoint1_kD;
        firstJointConfigs.Slot0.kG = Arm.FirstJoint.kArmJoint1_kG;
        firstJointConfigs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        firstJointConfigs.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
        firstJointConfigs.Voltage.withPeakForwardVoltage(Volts.of(Arm.FirstJoint.kArmJoint1_kPFV))
            .withPeakReverseVoltage(Volts.of(-Arm.FirstJoint.kArmJoint1_kPFV));
        firstJointConfigs.CurrentLimits.withSupplyCurrentLimit(Amps.of(Arm.FirstJoint.kArmJoint1_kSCL))
            .withSupplyCurrentLowerLimit(Amps.of(Arm.FirstJoint.kArmJoint1_kSCLL));
        firstJointConfigs.MotionMagic.MotionMagicCruiseVelocity = Arm.FirstJoint.kArmJoint1_MMCV;
        firstJointConfigs.MotionMagic.MotionMagicAcceleration = Arm.FirstJoint.kArmJoint1_MMA;
        firstJointConfigs.MotionMagic.MotionMagicJerk = Arm.FirstJoint.kArmJoint1_MMJ;
        firstJointConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        
        
        TalonFXConfiguration secondJointConfigs = new TalonFXConfiguration();
        secondJointConfigs.Slot0.kS = Arm.SecondJoint.kArmJoint2_kS;
        secondJointConfigs.Slot0.kV = Arm.SecondJoint.kArmJoint2_kV;
        secondJointConfigs.Slot0.kP = Arm.SecondJoint.kArmJoint2_kP;
        secondJointConfigs.Slot0.kI = Arm.SecondJoint.kArmJoint2_kI;
        secondJointConfigs.Slot0.kD = Arm.SecondJoint.kArmJoint2_kD;
        secondJointConfigs.Slot0.kG = Arm.SecondJoint.kArmJoint2_kG;
        secondJointConfigs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        secondJointConfigs.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
        secondJointConfigs.Voltage.withPeakForwardVoltage(Volts.of(Arm.SecondJoint.kArmJoint2_kPFV))
            .withPeakReverseVoltage(Volts.of(-Arm.SecondJoint.kArmJoint2_kPFV));
        secondJointConfigs.CurrentLimits.withSupplyCurrentLimit(Amps.of(Arm.SecondJoint.kArmJoint2_kSCL))
            .withSupplyCurrentLowerLimit(Amps.of(Arm.SecondJoint.kArmJoint2_kSCLL));
        secondJointConfigs.MotionMagic.MotionMagicCruiseVelocity = Arm.SecondJoint.kArmJoint2_MMCV;
        secondJointConfigs.MotionMagic.MotionMagicAcceleration = Arm.SecondJoint.kArmJoint2_MMA;
        secondJointConfigs.MotionMagic.MotionMagicJerk = Arm.SecondJoint.kArmJoint2_MMJ;
        secondJointConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        StatusCode statusFirstJoint = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            statusFirstJoint = m_armFirstJointMotor.getConfigurator().apply(firstJointConfigs);
            if (statusFirstJoint.isOK()) break;
        }
        if (!statusFirstJoint.isOK()) {
            System.out.println("Could not apply configs, error code: " + statusFirstJoint.toString());
        }

        StatusCode statusSecondJoint = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            statusSecondJoint = m_armSecondJointMotor.getConfigurator().apply(secondJointConfigs);
            if (statusSecondJoint.isOK()) break;
        }
        if (!statusSecondJoint.isOK()) {
            System.out.println("Could not apply configs, error code: " + statusSecondJoint.toString());
        }

        //m_armFirstJointMotorSim = m_armFirstJointMotor.getSimState();
        //m_armSecondJointMotorSim = m_armSecondJointMotor.getSimState();

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
        SmartDashboard.putData("Arm/J2OffsetChooser", m_offsetChooser);
        SmartDashboard.setPersistent("Arm/J2OffsetCooser");
        resetArmPositions();        
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_firstJointHalfcoder.periodic();
    m_secondJointHalfcoder.periodic();
    SmartDashboard.putNumber("Arm/J1-M-Pos", Units.rotationsToDegrees(m_armFirstJointMotor.getRotorPosition().getValueAsDouble())/Arm.FirstJoint.kArmReduction);
    SmartDashboard.putNumber("Arm/J2-M-Pos", Units.rotationsToDegrees(m_armSecondJointMotor.getRotorPosition().getValueAsDouble())/Arm.SecondJoint.kArmReduction);
    
    SmartDashboard.putNumber("Arm/J1-Angle", getFirstJointAngle());
    SmartDashboard.putNumber("Arm/J2-Angle", getSecondJointAngle());

    if(Robot.isReal())
    {
      firstJointAngle = m_firstJointHalfcoder.getAngle();
      secondJointAngle = m_secondJointHalfcoder.getAngle() - (firstJointAngle-180)*Arm.SecondJoint.kPulleyErrorRatio;
    }

    isArmReady = SmartDashboard.getBoolean("Arm/isArmReady", false);

    SmartDashboard.putBoolean("Arm/J1-isReached", isJ1GoalReached);
    SmartDashboard.putBoolean("Arm/J2-isReached", isJ2GoalReached);
    SmartDashboard.putNumber("Arm/J1LastAngle", armJ1LastAngle);
    SmartDashboard.putNumber("Arm/J2LastAngle", armJ2LastAngle);


    if(RobotState.isTest()) NeutralOutMotors();
    if(isInitialReady)
    {
      armJ1LastAngle = firstJointAngle;
      armJ1LastAngle = secondJointAngle;
    }

    armJ1MotorPos = Units.rotationsToDegrees(m_armFirstJointMotor.getRotorPosition().getValueAsDouble())/Arm.SecondJoint.kArmReduction;
    armJ2MotorPos = Units.rotationsToDegrees(m_armSecondJointMotor.getRotorPosition().getValueAsDouble())/Arm.SecondJoint.kArmReduction;

    if(!isMotorsSet)
    {
    if(Math.abs(SmartDashboard.getNumber("Arm/J1/M-StartPos", 0) - m_firstJointHalfcoder.getAngle()) > 2
        ||  Math.abs(SmartDashboard.getNumber("Arm/J2/M-StartPos", 0) - m_secondJointHalfcoder.getAngle()) > 2)
    {
      resetArmPositions();
    }
    else isMotorsSet = true;
    }

    if (m_offsetChooser.getSelected() < 10 && m_offsetChooser.getSelected() > -10) {
      if (m_offsetChooser != null) {
        j2Offset = m_offsetChooser.getSelected()*6; 
      } else j2Offset = 0;
    } else {
    if (m_offsetChooser.getSelected() > 10) j2Offset = 60;
    if (m_offsetChooser.getSelected() < -10) j2Offset = -60;
    }



  }

  @Override
  public void simulationPeriodic(){
    /* 
    m_armSimJ1.setInput(m_armFirstJointMotorSim.getMotorVoltage());
    m_armSimJ1.update(0.02);

    m_armSimJ2.setInput(m_armSecondJointMotorSim.getMotorVoltage());
    m_armSimJ2.update(0.02);

    m_firstJointHalfcoderSim.setJointAngle(Units.radiansToDegrees(m_armSimJ1.getAngleRads()) + (m_armSimJ1.getAngleRads()<0 ? 360 : 0));
    m_secondJointHalfcoderSim.setJointAngle(Units.radiansToDegrees(m_armSimJ2.getAngleRads()) + (m_armSimJ2.getAngleRads()<0 ? 360 : 0));

    m_armFirstJointMotorSim.setRawRotorPosition(Units.radiansToRotations(m_armSimJ1.getAngleRads())*Arm.FirstJoint.kArmReduction);
    m_armSecondJointMotorSim.setRawRotorPosition(Units.radiansToRotations(m_armSimJ2.getAngleRads())*Arm.SecondJoint.kArmReduction);


    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSimJ1.getCurrentDrawAmps() + m_armSimJ2.getCurrentDrawAmps()));
  
    SmartDashboard.putNumber("Arm Angle J1", Units.radiansToDegrees(m_armSimJ1.getAngleRads()));
    SmartDashboard.putNumber("Arm Angle J2", Units.radiansToDegrees(m_armSimJ2.getAngleRads()));

    firstJointAngle  = Units.radiansToDegrees(m_armSimJ1.getAngleRads());
    secondJointAngle = Units.radiansToDegrees(m_armSimJ2.getAngleRads());
    */
  }

  public Command reachGoalJ1Command(double angleJ1)
  {
    return run(() -> {
      reachGoalJ1(angleJ1);
    }).until(() -> { 
      return isJ1GoalReached;
    }).finallyDo(() -> brakeFirstJoint());
  }

  public Command reachGoalJ2Command(double angleJ2)
  {
    return run(() -> {
      reachGoalJ2(angleJ2, 0);
      
    }).until(() -> { 
      return isJ2GoalReached;
    }).finallyDo(() -> brakeFirstJoint());
  }

  public Command reachGoalCommand(double angleJ1, double angleJ2)
  {
    return run(() -> {
      
      reachGoal(angleJ1, angleJ2);
    }).until(() -> { 
      return isJ1GoalReached && isJ2GoalReached; 
    }).finallyDo(() -> brakeMotors());
  }

  public Command brakeMotorsCommand()
  {
    return runOnce(() -> brakeMotors());
  }

  public void reachGoal(double goalJ1, double goalJ2) 
  {
    if(isArmReady && !isInitialReady && SmartDashboard.getNumber("Arm/J2/M-StartPos", 0) > 330 && 
    SmartDashboard.getNumber("Arm/J1/M-StartPos", 0) > 160) 
            isInitialReady = true;
    if(isInitialReady)
    {
      if((goalJ1 > firstJointAngle &&  goalJ1 >= armJ1limitCCW) || firstJointAngle >= armJ1limitCCW + 8)
      {
        goalJ1 = armJ1limitCCW;
      }
      else if((goalJ1 < firstJointAngle &&  goalJ1 <= armJ1limitCW) || firstJointAngle <= armJ1limitCW - 8)
      {
        goalJ1 = armJ1limitCW;
      }

      SmartDashboard.putNumber("Arm/J1-Goal", goalJ1);
      isJ1GoalReached = (Math.abs(firstJointAngle - goalJ1) < Arm.FirstJoint.kAngleTolerance);

      m_armFirstJointMotor.setControl(m_firstJointMotionMagic.withPosition(Units.degreesToRotations(goalJ1)
      *Arm.FirstJoint.kArmReduction));

      if((goalJ2 > secondJointAngle &&  goalJ2 >= armJ2limitCCW) || secondJointAngle >= armJ2limitCCW + 4)
      {
        goalJ2 = armJ2limitCCW;
      }
      else if((goalJ2 < secondJointAngle &&  goalJ2 <= armJ2limitCW) || secondJointAngle <= armJ2limitCW - 4)
      {
        goalJ2 = armJ2limitCW;
      }    
  
      SmartDashboard.putNumber("Arm/J2-Goal", goalJ2);
  
      isJ2GoalReached = (Math.abs(secondJointAngle - goalJ2) < Arm.SecondJoint.kAngleTolerance);
  
      m_armSecondJointMotor.setControl(m_secondJointMotionMagic.withPosition(Units.degreesToRotations(goalJ2 + (goalJ1-180)*Arm.SecondJoint.kPulleyErrorRatio + j2Offset)
      *Arm.SecondJoint.kArmReduction));
    }
    else brakeMotors();  
  }


  public void reachGoalJ1(double goalJ1)
  {
    if((goalJ1 > firstJointAngle &&  goalJ1 >= armJ1limitCCW) || firstJointAngle >= armJ1limitCCW + 4)
    {
      goalJ1 = armJ1limitCCW;
    }
    else if((goalJ1 < firstJointAngle &&  goalJ1 <= armJ1limitCW) || firstJointAngle <= armJ1limitCW - 4)
    {
      goalJ1 = armJ1limitCW;
    }

    SmartDashboard.putNumber("Arm/J1-Goal", goalJ1);
    isJ1GoalReached = (Math.abs(firstJointAngle - goalJ1) < Arm.FirstJoint.kAngleTolerance);

    m_armFirstJointMotor.setControl(m_firstJointMotionMagic.withPosition(Units.degreesToRotations(goalJ1)
     *Arm.FirstJoint.kArmReduction));
  }

  public void reachGoalJ2(double goalJ2, double error)
  {
    if((goalJ2 > secondJointAngle &&  goalJ2 >= armJ2limitCCW) || secondJointAngle >= armJ2limitCCW + 4)
    {
      goalJ2 = armJ2limitCCW;
    }
    else if((goalJ2 < secondJointAngle &&  goalJ2 <= armJ2limitCW) || secondJointAngle <= armJ2limitCW - 4)
    {
      goalJ2 = armJ2limitCW;
    }    

    SmartDashboard.putNumber("Arm/J2-Goal", goalJ2);
    SmartDashboard.putNumber("Arm/J2-Err", error);

    isJ2GoalReached = (Math.abs(secondJointAngle - goalJ2) < Arm.SecondJoint.kAngleTolerance);

    m_armSecondJointMotor.setControl(m_secondJointMotionMagic.withPosition(Units.degreesToRotations(goalJ2 - error)
    *Arm.SecondJoint.kArmReduction));
  }

  public void brakeMotors()
  {
    brakeFirstJoint();
    brakeSecondJoint();
  }

  public void brakeFirstJoint(){
    m_armFirstJointMotor.setControl(m_firstJointBrake);
    
  }

  public void brakeSecondJoint(){
    m_armSecondJointMotor.setControl(m_secondJointBrake);
  }

  public void NeutralOutMotors()
  {
    neutralOutFirstJoint();
    neutralOutSecondJoint();
  }

  public void neutralOutFirstJoint(){
    m_armFirstJointMotor.setControl(m_firstJointNeutralOut);
  }

  public void neutralOutSecondJoint(){
    m_armSecondJointMotor.setControl(m_secondJointNeutralOut);
  }

  /* 
  public double getSimAngleJ1(){
    return Units.radiansToDegrees(m_armSimJ1.getAngleRads());
  }

  public double getSimAngleJ2(){
    return Units.radiansToDegrees(m_armSimJ2.getAngleRads());
  }
  */
  
  public double getFirstJointAngle(){
    return firstJointAngle;
  }

  public double getSecondJointAngle(){
    return secondJointAngle;
  }

  
  
  public void resetArmPositions(){
    // Units.degreesToRotations(m_firstJointHalfcoder.getAngle())*Arm.FirstJoint.kArmReduction
    
    m_armFirstJointMotor.setPosition(Units.degreesToRotations(getFirstJointAngle())*Arm.FirstJoint.kArmReduction);
    m_armSecondJointMotor.setPosition(Units.degreesToRotations(getSecondJointAngle())*Arm.SecondJoint.kArmReduction);
    SmartDashboard.putNumber("Arm/J2/M-StartPos", Units.rotationsToDegrees(m_armSecondJointMotor.getRotorPosition().getValueAsDouble())/Arm.SecondJoint.kArmReduction);
    SmartDashboard.putNumber("Arm/J1/M-StartPos", Units.rotationsToDegrees(m_armFirstJointMotor.getRotorPosition().getValueAsDouble())/Arm.FirstJoint.kArmReduction);
    armJ1LastAngle = m_firstJointHalfcoder.getAngle();
    armJ2LastAngle = m_secondJointHalfcoder.getAngle();
  }

  public void resetEncoders()
  {
    m_firstJointHalfcoder.reset();
    m_secondJointHalfcoder.reset();
  }

  public boolean isGoalReached()
  {
    return isJ1GoalReached && isJ2GoalReached;
  }

  public boolean checkMotorsSet()
  {
    if(!isMotorsSet) isMotorsSet = (Math.abs(m_firstJointHalfcoder.getAngle() - armJ1MotorPos)) < 2 && (Math.abs(m_secondJointHalfcoder.getAngle() - armJ2MotorPos)) < 2;
    return isMotorsSet;
  }

  public double getJ2Offset()
  {
    return j2Offset;
  }

  public void setJ2Offset(double offset)
  {
    j2Offset = offset;
  }

}


