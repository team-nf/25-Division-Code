// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.custom;

import javax.print.attribute.standard.PrinterInfo;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class ArmHalfEncoder {

    private DutyCycleEncoder m_encoder; 
    private boolean firstHalf = true;
    private double startAngle = 0.0;
    private double lastRawAngle = 0.0;
    private double jointAngle = 0.0;
    private double jointAngleHolder = 0.0;
    private double theId;

    public ArmHalfEncoder(int id, boolean isInverted, boolean isFirstHalf) {
        m_encoder = new DutyCycleEncoder(id);
        m_encoder.setInverted(isInverted);
        startAngle = Constants.InitialConstants.EncoderStartAngles[id];
        theId = id;
        firstHalf = isFirstHalf;
    }

    public double getAngle() {
        return jointAngle;
    }

    public double getRawEncoder() {
        return m_encoder.get()*360;
    }

    public void calcAngle()
    {
        if(Math.abs(lastRawAngle - getRawEncoder()) > 340) firstHalf = !firstHalf;
        lastRawAngle = getRawEncoder();
        jointAngleHolder = (getRawEncoder()/2 + (firstHalf ? 0 : 180)) - startAngle;
        if (jointAngleHolder < 0) jointAngleHolder += 360;
        jointAngle = jointAngleHolder;
    }

    public void periodic()
    {
        calcAngle();
        SmartDashboard.putNumber("Arm/HalfEncoder-" + Double.toString(theId + 1), jointAngle);
        SmartDashboard.putNumber("Arm/HalfRawEncoder-" + Double.toString(theId + 1), getRawEncoder());

    }

    public void reset() {
        firstHalf = false;
        lastRawAngle = getRawEncoder();
        SmartDashboard.putNumber("Arm/HalfEncoderStart-" + Double.toString(theId + 1), startAngle);
    }

    public DutyCycleEncoder getEncoder() {
        return m_encoder;
    }

    public double getStartAngle() {
        return startAngle;
    }
}
