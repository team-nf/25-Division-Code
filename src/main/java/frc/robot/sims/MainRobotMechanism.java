// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sims;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.Elevator;

/** Add your docs here. */
public class MainRobotMechanism {

    private final Mechanism2d mech;
    private final MechanismRoot2d root;
    private final MechanismLigament2d m_elevator;
    private final MechanismLigament2d m_joint1;
    private final MechanismLigament2d m_joint2;


    public MainRobotMechanism() {
        mech = new Mechanism2d(3, 3);
        root = mech.getRoot("main_system", 1.5, 0.5);
        m_elevator = root.append(new MechanismLigament2d("elevator", Elevator.kMinElevatorHeightMeters, 90));
        m_joint1 = m_elevator.append(
            new MechanismLigament2d("arm_joint_1", 0.345, -90, 5, new Color8Bit(Color.kPurple)));
        m_joint2 = m_joint1.append(
                new MechanismLigament2d("arm_joint_2", 0.345, -90, 5, new Color8Bit(Color.kSeaGreen)));

        SmartDashboard.putData("Mech2d", mech);
    }

    public void update(double l, double a, double b) {
        m_elevator.setLength(l);
        m_joint1.setAngle(180 - a);
        m_joint2.setAngle(180 - b);
    }
}
