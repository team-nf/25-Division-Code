// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

public class ClimbSubsystem extends SubsystemBase {
  
  // Motor CAN ID
  private static final int kMotorPort = 20;
  
  // Güvenlik limitleri
  private static final double kAmpLimit = 40.0;
  private static final double kVoltageLimit = 10.0;
  private static final double kMaxRotations = 480.0; // Maksimum dönüş sayısı
  
  // Sabit güç değeri
  private static final double kNormalPower = 0.5;
  
  // Motor ve kontroller
  private final TalonFX m_motor = new TalonFX(kMotorPort);
  private final TalonFXConfiguration m_config = new TalonFXConfiguration();
  private final StaticBrake m_brake = new StaticBrake();
  private final NeutralOut m_neutral = new NeutralOut();

  // Motor durumu
  private double m_currentRotations = 0;
  private double m_currentPower = 0;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    // Voltage ve akım limitleri
    m_config.Voltage.withPeakForwardVoltage(Volts.of(kVoltageLimit))
        .withPeakReverseVoltage(Volts.of(-kVoltageLimit));
    m_config.CurrentLimits.withSupplyCurrentLimit(kAmpLimit);

    // Motor yön ve fren ayarları
    m_config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    m_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Konfigürasyonu uygula
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_motor.getConfigurator().apply(m_config);
      if (status.isOK()) break;
    }
    
    if (!status.isOK()) {
      System.out.println("Climb motor konfigürasyonu yapılamadı: " + status.toString());
    }
    
    resetEncoder();
  }

  @Override
  public void periodic() {
    // Güncel dönüş değerini oku
    m_currentRotations = m_motor.getPosition().getValueAsDouble();
    
    // Telemetri gönder
    SmartDashboard.putNumber("Climb/Rotations", m_currentRotations);
    SmartDashboard.putNumber("Climb/Current", m_motor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Climb/Power", m_currentPower);
    
    // Test modunda veya robot devre dışıyken motoru durdur
    if (RobotState.isTest()) {
      m_motor.setControl(m_neutral);
    } else if (RobotState.isDisabled()) {
      m_motor.setControl(m_brake);
    }
  }
  
  /**
   * Motoru manuel olarak kontrol eder
   * @param power -1 ile 1 arası güç değeri
   */
  public void setPower(double power) {
    // Güvenlik kontrolleri
    if (m_currentRotations >= kMaxRotations && power > 0) {
      power = 0;
    }
    
    // Güç değerini kaydet
    m_currentPower = power;
    m_motor.set(power);
  }
  
  /**
   * Motoru durdurur
   */
  public void stop() {
    m_currentPower = 0;
    m_motor.setControl(m_brake);
  }
  
  /**
   * Encoder pozisyonunu sıfırlar
   */
  public void resetEncoder() {
    m_motor.setPosition(0);
  }
  
  /**
   * Rotasyon değerini döndürür
   */
  public double getRotations() {
    return m_currentRotations;
  }
  
  /**
   * Mevcut güç değerini döndürür
   */
  public double getCurrentPower() {
    return m_currentPower;
  }
  
  // KOMUTLAR
  
  /**
   * Sabit güç (+) ile yukarı çıkış komutu
   */
  public Command windCommand() {
    return run(() -> setPower(kNormalPower)).finallyDo(this::stop);
  }
  
  /**
   * Sabit güç (-) ile aşağı iniş komutu
   */
  public Command unwindCommand() {
    return run(() -> setPower(-kNormalPower)).finallyDo(this::stop);
  }
  
  /**
   * Motoru durdurma komutu
   */
  public Command stopCommand() {
    return runOnce(this::stop);
  }
  
  /**
   * Encoder'ı sıfırlama komutu
   */
  public Command resetEncoderCommand() {
    return runOnce(() -> resetEncoder());
  }
} 