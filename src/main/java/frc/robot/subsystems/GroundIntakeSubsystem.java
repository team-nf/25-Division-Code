// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntakeConstants;

public class GroundIntakeSubsystem extends SubsystemBase {

    // Motors
    private final TalonFX m_outerWheelsMotor = new TalonFX(GroundIntakeConstants.kOuterWheelsMotorID);
    private final TalonFX m_innerWheelsMasterMotor = new TalonFX(GroundIntakeConstants.kInnerWheelsMasterMotorID);
    private final TalonFX m_innerWheelsSlaveMotor = new TalonFX(GroundIntakeConstants.kInnerWheelsSlaveMotorID);
    private final TalonFX m_rotationMasterMotor = new TalonFX(GroundIntakeConstants.kRotationMasterMotorID);
    private final TalonFX m_rotationSlaveMotor = new TalonFX(GroundIntakeConstants.kRotationSlaveMotorID);

    // Sensor
    private final DigitalInput m_objectSensor = new DigitalInput(GroundIntakeConstants.kObjectSensorID);

    // Control Objects
    private final VelocityVoltage m_outerWheelVelocityControl = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage m_innerWheelVelocityControl = new VelocityVoltage(0).withSlot(0);
    private final MotionMagicVoltage m_rotationMotionMagic = new MotionMagicVoltage(0).withSlot(0);
    private final StaticBrake m_rotationBrake = new StaticBrake();

    // State tracking
    private boolean m_hasObject = false;
    private boolean m_isInnerWheelsRunning = false;
    private boolean m_isOuterWheelsRunning = false;
    private IntakeState m_currentState = IntakeState.STOWED;

    /** Enum for tracking the position state of the ground intake system */
    public enum IntakeState {
        STOWED,     // Inside the robot
        INTAKE,     // Normal intake position
        DROPPING    // Lower position for dropping objects
    }

    /** Creates a new GroundIntakeSubsystem. */
    public GroundIntakeSubsystem() {
        configureMotors();
    }

    private void configureMotors() {
        // Configure intake motors
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.Slot0.kP = GroundIntakeConstants.kIntakeVel_kP;
        intakeConfig.Slot0.kI = GroundIntakeConstants.kIntakeVel_kI;
        intakeConfig.Slot0.kD = GroundIntakeConstants.kIntakeVel_kD;
        intakeConfig.Slot0.kS = GroundIntakeConstants.kIntakeVel_kS;
        intakeConfig.Slot0.kV = GroundIntakeConstants.kIntakeVel_kV;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = GroundIntakeConstants.kCurrentLimit;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.Voltage.PeakForwardVoltage = GroundIntakeConstants.kVoltageLimit;
        intakeConfig.Voltage.PeakReverseVoltage = -GroundIntakeConstants.kVoltageLimit;

        m_outerWheelsMotor.getConfigurator().apply(intakeConfig);
        m_innerWheelsMasterMotor.getConfigurator().apply(intakeConfig);
        m_innerWheelsSlaveMotor.setControl(new Follower(m_innerWheelsMasterMotor.getDeviceID(), false));
        m_innerWheelsSlaveMotor.setNeutralMode(NeutralModeValue.Coast);

        // Configure rotation motors
        TalonFXConfiguration rotationConfig = new TalonFXConfiguration();
        rotationConfig.Slot0.kP = GroundIntakeConstants.kRotationPos_kP;
        rotationConfig.Slot0.kI = GroundIntakeConstants.kRotationPos_kI;
        rotationConfig.Slot0.kD = GroundIntakeConstants.kRotationPos_kD;
        rotationConfig.Slot0.kS = GroundIntakeConstants.kRotationPos_kS;
        rotationConfig.Slot0.kG = GroundIntakeConstants.kRotationPos_kG;
        rotationConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rotationConfig.CurrentLimits.SupplyCurrentLimit = GroundIntakeConstants.kCurrentLimit;
        rotationConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rotationConfig.Voltage.PeakForwardVoltage = GroundIntakeConstants.kVoltageLimit;
        rotationConfig.Voltage.PeakReverseVoltage = -GroundIntakeConstants.kVoltageLimit;

        // Motion Magic settings for rotation
        rotationConfig.MotionMagic.MotionMagicCruiseVelocity = GroundIntakeConstants.kRotationMMCV;
        rotationConfig.MotionMagic.MotionMagicAcceleration = GroundIntakeConstants.kRotationMMA;
        rotationConfig.MotionMagic.MotionMagicJerk = GroundIntakeConstants.kRotationMMJ;

        m_rotationMasterMotor.getConfigurator().apply(rotationConfig);
        m_rotationSlaveMotor.setControl(new Follower(m_rotationMasterMotor.getDeviceID(), false));
        m_rotationSlaveMotor.setNeutralMode(NeutralModeValue.Brake);

        // Reset positions on startup
        m_rotationMasterMotor.setPosition(0);
    }

    /**
     * Sets the state of the ground intake system.
     * 
     * @param state The target state for the intake system
     * @return A command that changes the intake state
     */
    public Command setIntakeStateCommand(IntakeState state) {
        return new CommandBase() {
            @Override
            public void initialize() {
                setIntakeState(state);
            }

            @Override
            public boolean isFinished() {
                // Check if we're close to the target angle
                double currentAngleDegrees = getRotationAngleDegrees();
                double targetAngleDegrees = getAngleForState(state);
                return Math.abs(currentAngleDegrees - targetAngleDegrees) < GroundIntakeConstants.kAngleToleranceDegrees;
            }

            @Override
            public void end(boolean interrupted) {
                m_rotationMasterMotor.setControl(m_rotationBrake);
            }
        };
    }

    /**
     * Sets the state of the ground intake system.
     * 
     * @param state The target state for the intake system
     */
    public void setIntakeState(IntakeState state) {
        double targetAngleDegrees = getAngleForState(state);
        setRotationAngle(targetAngleDegrees);
        m_currentState = state;
        SmartDashboard.putString("GroundIntake/CurrentState", state.toString());
    }

    /**
     * Gets the angle in degrees for a given state
     */
    private double getAngleForState(IntakeState state) {
        switch (state) {
            case STOWED:
                return GroundIntakeConstants.kStartPositionDegrees;
            case INTAKE:
                return GroundIntakeConstants.kIntakePositionDegrees;
            case DROPPING:
                return GroundIntakeConstants.kDropPositionDegrees;
            default:
                return GroundIntakeConstants.kStartPositionDegrees;
        }
    }

    /**
     * Sets the rotation angle of the ground intake system.
     * 
     * @param angleDegrees Target angle in degrees
     * @return A command that moves the intake to the specified angle
     */
    public Command setRotationAngleCommand(double angleDegrees) {
        return new CommandBase() {
            @Override
            public void initialize() {
                setRotationAngle(angleDegrees);
            }

            @Override
            public boolean isFinished() {
                double currentAngleDegrees = getRotationAngleDegrees();
                return Math.abs(currentAngleDegrees - angleDegrees) < GroundIntakeConstants.kAngleToleranceDegrees;
            }

            @Override
            public void end(boolean interrupted) {
                m_rotationMasterMotor.setControl(m_rotationBrake);
            }
        };
    }

    /**
     * Sets the rotation angle of the ground intake system.
     * 
     * @param angleDegrees Target angle in degrees
     */
    public void setRotationAngle(double angleDegrees) {
        // Clamp the angle to the allowed range
        double clampedAngle = Math.max(GroundIntakeConstants.kMinAngleDegrees,
                Math.min(angleDegrees, GroundIntakeConstants.kMaxAngleDegrees));
        
        // Convert from degrees to rotations for the motor controller
        double rotations = degreesToRotations(clampedAngle);
        
        // Set the motor position
        m_rotationMasterMotor.setControl(m_rotationMotionMagic.withPosition(rotations));
        
        SmartDashboard.putNumber("GroundIntake/TargetAngleDegrees", clampedAngle);
    }

    /**
     * Gets the current rotation angle of the intake system.
     * 
     * @return Current angle in degrees
     */
    public double getRotationAngleDegrees() {
        double rotations = m_rotationMasterMotor.getPosition().getValueAsDouble();
        return rotationsToDegrees(rotations);
    }

    /**
     * Converts an angle in degrees to rotations for the motor controller.
     * Accounts for gear ratio.
     * 
     * @param degrees Angle in degrees
     * @return Equivalent in motor rotations
     */
    private double degreesToRotations(double degrees) {
        // 360 degrees = 1 rotation of the mechanism
        // We need to account for the gear ratio (motor rotations per mechanism rotation)
        return (degrees / 360.0) * GroundIntakeConstants.kRotationGearRatio;
    }

    /**
     * Converts rotations (from motor controller) to degrees.
     * Accounts for gear ratio.
     * 
     * @param rotations Motor rotations
     * @return Equivalent angle in degrees
     */
    private double rotationsToDegrees(double rotations) {
        // Convert motor rotations to mechanism rotations, then to degrees
        return (rotations / GroundIntakeConstants.kRotationGearRatio) * 360.0;
    }

    /**
     * Sets the intake wheels to run at the appropriate speed based on the chassis speed,
     * making the object stationary relative to the field for easier intake.
     * 
     * @param chassisSpeedCmPerSec Current forward speed of the chassis in cm/s.
     * @param useInnerWheels Whether to use inner wheels as well or just outer wheels.
     * @return A command that runs the intake wheels until an object is detected
     */
    public Command intakeObjectCommand(double chassisSpeedCmPerSec, boolean useInnerWheels) {
        return new CommandBase() {
            @Override
            public void initialize() {
                setIntakeWheelsVelocity(chassisSpeedCmPerSec, useInnerWheels);
                addRequirements(GroundIntakeSubsystem.this);
            }

            @Override
            public boolean isFinished() {
                return m_hasObject;
            }

            @Override
            public void end(boolean interrupted) {
                if (interrupted) {
                    stopIntakeWheels();
                }
            }
        };
    }

    /**
     * Sets the intake wheels to run at the appropriate speed to eject an object.
     * 
     * @param useInnerWheels Whether to use inner wheels as well or just outer wheels.
     * @return A command that runs the intake wheels in reverse until an object is no longer detected
     */
    public Command ejectObjectCommand(boolean useInnerWheels) {
        return new CommandBase() {
            @Override
            public void initialize() {
                // Reverse direction for ejecting
                setIntakeWheelsVelocity(-500, useInnerWheels); // Example speed: 5 m/s
                addRequirements(GroundIntakeSubsystem.this);
            }

            @Override
            public boolean isFinished() {
                return !m_hasObject;
            }

            @Override
            public void end(boolean interrupted) {
                stopIntakeWheels();
            }
        };
    }

    /**
     * Sets the intake wheels to run at the appropriate speed based on the chassis speed.
     * 
     * @param chassisSpeedCmPerSec Current forward speed of the chassis in cm/s.
     * @param useInnerWheels Whether to use inner wheels as well or just outer wheels.
     */
    public void setIntakeWheelsVelocity(double chassisSpeedCmPerSec, boolean useInnerWheels) {
        // The speed of the object relative to the field should be 0 for optimal intake
        // This means the wheels need to move at the same speed as the chassis, but in opposite direction
        double targetObjectSpeedCmPerSec = chassisSpeedCmPerSec;
        
        // Clamp the speed to reasonable limits
        targetObjectSpeedCmPerSec = Math.max(GroundIntakeConstants.kMinIntakeObjectSpeedCmPerSec,
                Math.min(Math.abs(targetObjectSpeedCmPerSec), GroundIntakeConstants.kMaxIntakeObjectSpeedCmPerSec));
        
        // If the chassis is moving backward, we might need to adjust the direction
        if (chassisSpeedCmPerSec < 0) {
            targetObjectSpeedCmPerSec = -targetObjectSpeedCmPerSec;
        }

        // Convert cm/s to rotations per second
        double outerWheelCircumferenceCm = Math.PI * GroundIntakeConstants.kOuterWheelDiameterCm;
        double innerWheelCircumferenceCm = Math.PI * GroundIntakeConstants.kInnerWheelDiameterCm;
        
        // RPS = (cm/s) / (cm/rotation) * gear ratio
        double outerMotorRPS = (targetObjectSpeedCmPerSec / outerWheelCircumferenceCm) * GroundIntakeConstants.kIntakeGearRatio;
        double innerMotorRPS = (targetObjectSpeedCmPerSec / innerWheelCircumferenceCm) * GroundIntakeConstants.kIntakeGearRatio;

        // Set motor velocities
        m_outerWheelsMotor.setControl(m_outerWheelVelocityControl.withVelocity(outerMotorRPS));
        m_isOuterWheelsRunning = true;
        
        if (useInnerWheels) {
            m_innerWheelsMasterMotor.setControl(m_innerWheelVelocityControl.withVelocity(innerMotorRPS));
            m_isInnerWheelsRunning = true;
        } else {
            m_innerWheelsMasterMotor.stopMotor();
            m_isInnerWheelsRunning = false;
        }

        SmartDashboard.putNumber("GroundIntake/TargetSpeedCmPerSec", targetObjectSpeedCmPerSec);
        SmartDashboard.putNumber("GroundIntake/OuterTargetRPS", outerMotorRPS);
        SmartDashboard.putNumber("GroundIntake/InnerTargetRPS", innerMotorRPS);
        SmartDashboard.putBoolean("GroundIntake/UsingInnerWheels", useInnerWheels);
    }

    /** Stops the intake wheel motors. */
    public void stopIntakeWheels() {
        m_outerWheelsMotor.stopMotor();
        m_innerWheelsMasterMotor.stopMotor();
        m_isOuterWheelsRunning = false;
        m_isInnerWheelsRunning = false;
    }

    /** Stops the rotation motors and engages brake mode. */
    public void stopRotation() {
        m_rotationMasterMotor.setControl(m_rotationBrake);
    }

    /** Returns whether an object is currently detected in the intake. */
    public boolean hasObject() {
        return m_hasObject;
    }

    /** Returns the current state of the intake system. */
    public IntakeState getCurrentState() {
        return m_currentState;
    }

    @Override
    public void periodic() {
        // Read the object sensor (inverted if sensor returns true when no object present)
        boolean objectDetected = !m_objectSensor.get();
        
        // Only set m_hasObject to true if it wasn't already true (debounce)
        if (objectDetected && !m_hasObject) {
            m_hasObject = true;
            // If we detect an object and wheels are running, stop them
            if (m_isInnerWheelsRunning && m_isOuterWheelsRunning) {
                stopIntakeWheels();
            }
        } else if (!objectDetected && m_hasObject) {
            m_hasObject = false;
        }

        // Update dashboard with current values organized by category
        
        // Object state
        SmartDashboard.putBoolean("GroundIntake/State/HasObject", m_hasObject);
        SmartDashboard.putString("GroundIntake/State/IntakeState", m_currentState.toString());
        SmartDashboard.putBoolean("GroundIntake/State/ObjectSensorRaw", !m_objectSensor.get());
        
        // Rotation subsystem
        SmartDashboard.putNumber("GroundIntake/Rotation/AngleDegrees", getRotationAngleDegrees());
        SmartDashboard.putNumber("GroundIntake/Rotation/RawEncoderRotations", m_rotationMasterMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("GroundIntake/Rotation/RawSlaveEncoderRotations", m_rotationSlaveMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("GroundIntake/Rotation/TargetAngleDegrees", SmartDashboard.getNumber("GroundIntake/TargetAngleDegrees", 0));
        SmartDashboard.putNumber("GroundIntake/Rotation/ErrorDegrees", 
            getRotationAngleDegrees() - SmartDashboard.getNumber("GroundIntake/TargetAngleDegrees", 0));
        SmartDashboard.putNumber("GroundIntake/Rotation/MasterCurrentAmps", m_rotationMasterMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("GroundIntake/Rotation/SlaveCurrentAmps", m_rotationSlaveMotor.getSupplyCurrent().getValueAsDouble());
        
        // Outer wheels subsystem
        double outerRPS = m_outerWheelsMotor.getVelocity().getValueAsDouble();
        double outerWheelCircumferenceCm = Math.PI * GroundIntakeConstants.kOuterWheelDiameterCm;
        double outerLinearSpeedCmPerSec = (outerRPS / GroundIntakeConstants.kIntakeGearRatio) * outerWheelCircumferenceCm;
        
        SmartDashboard.putNumber("GroundIntake/OuterWheels/RPS", outerRPS);
        SmartDashboard.putNumber("GroundIntake/OuterWheels/LinearSpeedCmPerSec", outerLinearSpeedCmPerSec);
        SmartDashboard.putNumber("GroundIntake/OuterWheels/RawEncoderPosition", m_outerWheelsMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("GroundIntake/OuterWheels/TargetRPS", SmartDashboard.getNumber("GroundIntake/OuterTargetRPS", 0));
        SmartDashboard.putNumber("GroundIntake/OuterWheels/CurrentAmps", m_outerWheelsMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("GroundIntake/OuterWheels/VoltageOut", m_outerWheelsMotor.getMotorVoltage().getValueAsDouble());
        
        // Inner wheels subsystem
        double innerRPS = m_innerWheelsMasterMotor.getVelocity().getValueAsDouble();
        double innerWheelCircumferenceCm = Math.PI * GroundIntakeConstants.kInnerWheelDiameterCm;
        double innerLinearSpeedCmPerSec = (innerRPS / GroundIntakeConstants.kIntakeGearRatio) * innerWheelCircumferenceCm;
        
        SmartDashboard.putNumber("GroundIntake/InnerWheels/MasterRPS", innerRPS);
        SmartDashboard.putNumber("GroundIntake/InnerWheels/LinearSpeedCmPerSec", innerLinearSpeedCmPerSec);
        SmartDashboard.putNumber("GroundIntake/InnerWheels/MasterRawEncoderPosition", m_innerWheelsMasterMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("GroundIntake/InnerWheels/SlaveRawEncoderPosition", m_innerWheelsSlaveMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("GroundIntake/InnerWheels/TargetRPS", SmartDashboard.getNumber("GroundIntake/InnerTargetRPS", 0));
        SmartDashboard.putNumber("GroundIntake/InnerWheels/MasterCurrentAmps", m_innerWheelsMasterMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("GroundIntake/InnerWheels/SlaveCurrentAmps", m_innerWheelsSlaveMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("GroundIntake/InnerWheels/VoltageOut", m_innerWheelsMasterMotor.getMotorVoltage().getValueAsDouble());
        
        // Status tracking
        SmartDashboard.putBoolean("GroundIntake/Status/InnerWheelsRunning", m_isInnerWheelsRunning);
        SmartDashboard.putBoolean("GroundIntake/Status/OuterWheelsRunning", m_isOuterWheelsRunning);
    }
} 