// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.custom;

import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;

/** Add your docs here. */
public class ArmHalfEncoderSim {

    private ArmHalfEncoder m_armEncoder; 
    private DutyCycleEncoderSim m_encoderSim;

    public ArmHalfEncoderSim(ArmHalfEncoder encoder) {
        m_armEncoder = encoder;
        m_encoderSim = new DutyCycleEncoderSim(m_armEncoder.getEncoder());
    }

    public void setJointAngle(double angle) {
        angle = angle + m_armEncoder.getStartAngle();
        if (angle > 360) angle -= 360;
        m_encoderSim.set((angle % 180)*2);
    }

}
