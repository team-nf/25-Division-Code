// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

import static edu.wpi.first.units.Units.*;


public class GripperSubsystem extends SubsystemBase {

  private final TalonFX the_hupletici = new TalonFX(GripperConstants.kGripperID);

  private boolean hasAlgae = false;
  private boolean hasCoral = false;

  private int timerA_take = 0;
  private int timerC_take = 0;
  private int timerA_throw = 0;
  private int timerC_throw = 0;

  private final int algaeDelay = 20;

  private final DigitalInput m_AlgaeSensor = new DigitalInput(GripperConstants.kAlgaeSensor);
  private final DigitalInput m_coralSensor = new DigitalInput(GripperConstants.kCoralSensor);

  private double autoCoralCounter = 0;

  private boolean isCoralStuck = false;

  private final SendableChooser<Boolean> m_sensorUseSelect  = new SendableChooser<>();
  private final SendableChooser<Boolean> m_hasAlgaeManual       = new SendableChooser<>();
  private final SendableChooser<Boolean> m_hasCoralManual       = new SendableChooser<>();

  private boolean manualSensorUsage = false;
  private boolean hasAlgaeManual = false;
  private boolean hasCoralManual = false;

  /** 
   * Creates a new GripperSubsystem. 
   * Valla tuna ercan senden kopyaladım.
   */
  public GripperSubsystem() {
    
    m_sensorUseSelect.setDefaultOption("Real", false);
    m_sensorUseSelect.addOption("Manual", true);
    m_sensorUseSelect.onChange(this::setManualSensorUsage);

    m_hasAlgaeManual.setDefaultOption("Yes", true);
    m_hasAlgaeManual.addOption("No", false);
    m_hasAlgaeManual.onChange(this::setHasAlgaeManual);

    m_hasCoralManual.setDefaultOption("Yes", true);
    m_hasCoralManual.addOption("No", false);
    m_hasCoralManual.onChange(this::setHasCoralManual);


  }

  private void setManualSensorUsage(boolean m)
  {
    manualSensorUsage = m;
  }

  public void setHasAlgaeManual(boolean m)
  {
    hasAlgaeManual = m;
  }

  public void setHasCoralManual(boolean m)
  {
    hasCoralManual = m;
  }

  public boolean hasAlgae() {return (!manualSensorUsage ? this.hasAlgae : hasAlgaeManual);}

  public boolean hasCoral(){return (!manualSensorUsage ? this.hasCoral : hasCoralManual);}

  public boolean hasNotCoral() {return !this.hasCoral;}


  /** Used for testing */
  //public Command controlWithTriggers(double input) {return run(() -> );}
  
  //public Command takeAlgae() {return runEnd(() -> sparkPID.setReference(.6, ControlType.kMAXMotionVelocityControl), this::stop).until(this::hasAlgae);}
  public Command takeAlgae() {return run(() -> the_hupletici.set(.75)).until(this::hasAlgae).finallyDo(() -> {if(hasAlgae()) the_hupletici.set(0.2); else stop();});}

  //public Command takeCoral() {return runEnd(() -> sparkPID.setReference(-0.3, ControlType.kMAXMotionVelocityControl), this::stop).until(this::hasCoral);}
  public Command takeCoral() {return run(() -> the_hupletici.set(-0.43)).until(this::hasCoral).finallyDo(this::stop);}

  //public Command throwAlgae() {return runEnd(() -> sparkPID.setReference(-0.6, ControlType.kMAXMotionVelocityControl), this::stop);}
  public Command throwAlgae() {return run(() -> the_hupletici.set(-0.5)).finallyDo(this::stop);}

  //public Command throwCoral() {return runEnd(() -> sparkPID.setReference(0.5, ControlType.kMAXMotionVelocityControl), this::stop);}
  public Command throwCoral() {return runEnd(() -> the_hupletici.set(-0.6), this::stop).onlyWhile(this::hasCoral);}
  //public Command stop() {return run(() -> sparkPID.setReference(0.03 ControlType.kMAXMotionVelocityControl));}
  public void stop() {the_hupletici.stopMotor(); autoCoralCounter = 0;}

  public Command runGripper(double speed) {return run(() -> the_hupletici.set(speed)).finallyDo(this::stop);}

  public Command stopCommand() {return run((() -> the_hupletici.stopMotor()));}

  public Command TakeCoralAutoCommand()
  {
    return run(this::AutoCoral).until(this::hasCoral).finallyDo(this::stop);
  }

  public Command ThrowCoralAutoCommand()
  {
    return run(() -> the_hupletici.set(-0.6)).until(this::hasNotCoral).finallyDo(this::stop);
  }

  /** 
  public void AutoCoral()
  {
    autoCoralCounter ++;
    if(autoCoralCounter <= 70) the_hupletici.set(-0.3);
    else if(autoCoralCounter <= 72) the_hupletici.stopMotor();
    else autoCoralCounter = 0;

    if(the_hupletici.getTorqueCurrent() > 6)
    {
      the_hupletici.set(-0.3);
      autoCoralCounter = 0;
    }

  } **/

  public void AutoCoral()
  {
    if(isCoralStuck)
    {
      autoCoralCounter++;
      if(autoCoralCounter > 1 && the_hupletici.getSupplyCurrent().getValueAsDouble() < 12.5)
      {
        isCoralStuck = false;
        autoCoralCounter = 0;
      }
      else the_hupletici.set(0.15);
    }
    else
    {
      the_hupletici.set(-0.3);
      if(the_hupletici.getSupplyCurrent().getValueAsDouble() > 20) isCoralStuck = true;
    }

  }
  
  @Override
  public void periodic() {

    // This method will be called once per scheduler run

    // iğrenç şeyler yaptım -yüşa
    
    if(!m_AlgaeSensor.get() && !hasAlgae) {
      timerA_take++;
      if (!m_AlgaeSensor.get() && timerA_take == algaeDelay) // periodic 20msde bir çağrılıyor, 1 saniye beklemek için 50 çağrı yapılmalı
      {
        this.hasAlgae = true;
        timerA_take = 0;
      };
    } else {
      if (timerA_take != 0) timerA_take = 0;
    }

    if(m_AlgaeSensor.get() && hasAlgae) {
      timerA_throw++;
      if (m_AlgaeSensor.get() && timerA_throw == algaeDelay) // periodic 20msde bir çağrılıyor, 1 saniye beklemek için 50 çağrı yapılmalı
      {
        this.hasAlgae = false;
        timerA_throw = 0;
      };
    } else {
      if (timerA_throw != 0) timerA_throw = 0;
    }

    hasCoral = !m_coralSensor.get();

    // Telemetry
    //SmartDashboard.putNumber("Current Velocity", the_hupletici.getVelocity().getValueAsDouble());

    SmartDashboard.putBoolean("Has Coral?: ", hasCoral);
    SmartDashboard.putBoolean("Has Algae?: ", hasAlgae);
  }
}