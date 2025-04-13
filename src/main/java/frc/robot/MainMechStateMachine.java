// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.StatePositions;
import frc.robot.sims.MainRobotMechanism;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;

/** Add your docs here. */
public class MainMechStateMachine {

    /** Creates a new MainMechSubsystem. */
    private final ArmSubsystem m_armSubsystem;
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final GripperSubsystem m_gripperSubsystem;
    private final MainRobotMechanism m_mechanism = new MainRobotMechanism();

    StructPublisher<Pose3d> elevatorStage0pub = NetworkTableInstance.getDefault()
            .getStructTopic("3dSim/eleStage0", Pose3d.struct).publish();
    StructPublisher<Pose3d> elevatorStage1pub = NetworkTableInstance.getDefault()
            .getStructTopic("3dSim/eleStage1", Pose3d.struct).publish();
    StructPublisher<Pose3d> armStage0pub = NetworkTableInstance.getDefault()
            .getStructTopic("3dSim/armStage0", Pose3d.struct).publish();
    StructPublisher<Pose3d> armStage1pub = NetworkTableInstance.getDefault()
            .getStructTopic("3dSim/armStage1", Pose3d.struct).publish();

    StructPublisher<Pose3d> fakeRobotPose = NetworkTableInstance.getDefault()
            .getStructTopic("3dSim/fakeRobots", Pose3d.struct).publish();

    private double eleGeneralHeight = 0;
    private double eleStage0Height = 0;
    private double eleStage1Height = 0;
    private double armJ1Angle = 0;
    private double armJ2Angle = 0;

    private String lastState = "FullyClosed";
    private boolean isGoalReached = false;
    private List<String> algaeStateList = Arrays.asList(
        "ThrowAlgaeNet", "ThrowAlgaeProcessor", "AlgaeGround", "Algae23", "Algae34", "AlgaeCarry", "AlgaeGroundToCarry", "Algae34ToCarry");

    private List<String> coralStateList = Arrays.asList(
        "FullyClosed", "Closed", "CoralIntake", "CoralCarry", "CoralStage1", "CoralStage2", "CoralStage3", "CoralStage4");

    public MainMechStateMachine(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, GripperSubsystem gripperSubsystem) {
        m_armSubsystem = armSubsystem;
        m_elevatorSubsystem = elevatorSubsystem;
        m_gripperSubsystem = gripperSubsystem;
    }

    public void MechStateControl(String state) {
        /*
        if (!RobotState.isAutonomous()) {
            if (lastState == "FullyClosed" && state != "Closed") state = "FullyClosed";
            else if (lastState != "Closed" && state == "FullyClosed") state = "Closed";
        }
        */
        
        if (lastState == "ThrowAlgaeNet" && state != "ThrowAlgaeNet") state = "AlgaeCarry";

        if (!(lastState == "FullyClosed" || lastState == "CoralIntake") && state == "CoralIntake") state = "FullyClosed";

        if (lastState == "CoralIntake" && !(state == "FullyClosed" || state == "CoralIntake")) state = "FullyClosed";

        if (lastState == "Algae34" && state != "Algae34") state = "Algae34ToCarry";

        if (lastState == "AlgaeGround" && state != "AlgaeGround") state = "AlgaeGroundToCarry";

        if (lastState == "CoralStage4" && state != "CoralStage4") state = "Stage4ToClosed";
        

        if (m_gripperSubsystem.hasAlgae()) {
            if (!algaeStateList.contains(state)) {
                state = lastState;
            }
        }

        /*if (m_gripperSubsystem.hasCoral()) {
            if (!coralStateList.contains(state)) {
                state = lastState;
            }
        }*/

        lastState = state;
        switch (state) {
            case "CoralIntake":
                CoralIntake();
                break;
            case "CoralCarry":
                CoralCarry();
                break;
            case "CoralStage1":
                CoralStage1();
                break;
            case "CoralStage2":
                CoralStage2();
                break;
            case "CoralStage3":
                CoralStage3();
                break;
            case "CoralStage4":
                CoralStage4();
                break;
            case "Stage4ToClosed":
                Stage4ToClosed();
                break;
            case "CoralStage4Pre":
                CoralStage4Pre();
                break;
            case "ThrowAlgaeNet":
                ThrowAlgaeNet();
                break;
            case "ThrowAlgaeProcessor":
                ThrowAlgaeProcessor();
                break;
            case "AlgaeGround":
                AlgaeGround();
                break;
            case "AlgaeGroundToCarry":
                AlgaeGroundToCarry();
                break;
            case "Algae23":
                Algae23();
                break;
            case "Algae34":
                Algae34();
                break;
            case "Algae34ToCarry":
                Algae34ToCarry();
                break;
            case "AlgaeCarry":
                AlgaeCarry();
                break;
            case "FullyClosed":
                FullyClosed();
                break;
            case "Closed":
                Closed();
                break;
            default:
                AlgaeCarry();
                break;
        }
    }

    public void reachGoal(double height, double angleJ1, double angleJ2) {
        m_elevatorSubsystem.reachGoal(height);
        if(lastState != "CoralStage4" || m_elevatorSubsystem.getElevatorHeight() > Elevator.minHeightForL4) 
                m_armSubsystem.reachGoal(angleJ1, angleJ2);
        isGoalReached = m_armSubsystem.isGoalReached() && m_elevatorSubsystem.isGoalReached();
    }

    public void CoralIntake() {
        reachGoal(StatePositions.CoralIntake[0], StatePositions.CoralIntake[1], StatePositions.CoralIntake[2]);
    }

    public void CoralStage1() {
        reachGoal(StatePositions.CoralStage1[0], StatePositions.CoralStage1[1], StatePositions.CoralStage1[2]);
    }

    public void CoralStage2() {
        reachGoal(StatePositions.CoralStage2[0], StatePositions.CoralStage2[1], StatePositions.CoralStage2[2]);
    }

    public void CoralStage3() {
        reachGoal(StatePositions.CoralStage3[0], StatePositions.CoralStage3[1], StatePositions.CoralStage3[2]);
    }

    public void CoralStage4() {
        reachGoal(StatePositions.CoralStage4[0], StatePositions.CoralStage4[1], StatePositions.CoralStage4[2]);
    }

    public void CoralStage4Pre() {
        reachGoal(StatePositions.CoralStage4Pre[0], StatePositions.CoralStage4Pre[1], StatePositions.CoralStage4Pre[2]);
    }

    public void Stage4ToClosed() {
        reachGoal(StatePositions.CoralStage4[0], StatePositions.Closed[1], StatePositions.Closed[2]);
    }

    public void CoralCarry() {
        reachGoal(StatePositions.CoralCarry[0], StatePositions.CoralCarry[1], StatePositions.CoralCarry[2]);
    }

    public void ThrowAlgaeNet() {
        reachGoal(StatePositions.AlgaeThrowNet[0], StatePositions.AlgaeThrowNet[1], StatePositions.AlgaeThrowNet[2]);
    }

    public void ThrowAlgaeProcessor() {
        reachGoal(StatePositions.AlgaeThrowProcessor[0], StatePositions.AlgaeThrowProcessor[1], StatePositions.AlgaeThrowProcessor[2]);
    }

    public void AlgaeGround() {
        reachGoal(StatePositions.AlgaeGround[0], StatePositions.AlgaeGround[1], StatePositions.AlgaeGround[2]);
    }

    public void AlgaeGroundToCarry() {
        reachGoal(StatePositions.AlgaeGround[0], StatePositions.AlgaeCarry[1], StatePositions.AlgaeCarry[2]);
    }

    public void Algae23() {
        reachGoal(StatePositions.AlgaeStage23[0], StatePositions.AlgaeStage23[1], StatePositions.AlgaeStage23[2]);
    }

    public void Algae23FromL4() {
        reachGoal(StatePositions.AlgaeStage23FromL4[0], StatePositions.AlgaeStage23FromL4[1], StatePositions.AlgaeStage23FromL4[2]);
    }

    public void Algae34() {
        reachGoal(StatePositions.AlgaeStage34[0], StatePositions.AlgaeStage34[1], StatePositions.AlgaeStage34[2]);
    }

    public void Algae34FromL4() {
        reachGoal(StatePositions.AlgaeStage34FromL4[0], StatePositions.AlgaeStage34FromL4[1], StatePositions.AlgaeStage34FromL4[2]);
    }

    public void Algae34ToCarry() {
        reachGoal(StatePositions.AlgaeStage34[0], StatePositions.AlgaeCarry[1], StatePositions.AlgaeCarry[2]);
    }

    public void AlgaeCarry() {
        reachGoal(StatePositions.AlgaeCarry[0], StatePositions.AlgaeCarry[1], StatePositions.AlgaeCarry[2]);
    }

    public void Closed() {
        reachGoal(StatePositions.Closed[0], StatePositions.Closed[1], StatePositions.Closed[2]);
    }

    public void FullyClosed() {
        reachGoal(StatePositions.FullyClosed[0], StatePositions.FullyClosed[1], StatePositions.FullyClosed[2]);
    }

    public void updateMech() {
        eleStage0Height = (eleGeneralHeight > Constants.Elevator.kStage1Height ? eleGeneralHeight - Constants.Elevator.kStage1Height : 0);
        eleStage1Height = eleGeneralHeight - eleStage0Height;
        armJ1Angle = m_armSubsystem.getFirstJointAngle();
        armJ2Angle = m_armSubsystem.getSecondJointAngle();

        elevatorStage0pub.set(new Pose3d(0, 0, eleStage0Height, new Rotation3d(0, 0, 0)));

        elevatorStage1pub.set(new Pose3d(0, 0, eleGeneralHeight, new Rotation3d(0, 0, 0)));

        armStage0pub.set(new Pose3d(Constants.Arm.FirstJoint.kSimOffsets[0], Constants.Arm.FirstJoint.kSimOffsets[1], Constants.Arm.FirstJoint.kSimOffsets[2] + eleGeneralHeight,
                new Rotation3d(0, Units.degreesToRadians(180 - armJ1Angle), 0)));
        armStage1pub.set(new Pose3d(Constants.Arm.SecondJoint.kSimOffsets[0] + Constants.Arm.FirstJoint.kArmLength * Math.sin(Units.degreesToRadians(180 - armJ1Angle)),
                Constants.Arm.SecondJoint.kSimOffsets[1],
                Constants.Arm.SecondJoint.kSimOffsets[2] + Constants.Arm.FirstJoint.kArmLength * Math.cos(Units.degreesToRadians(180 - armJ1Angle)) + eleGeneralHeight,
                new Rotation3d(0, Units.degreesToRadians((180 - armJ1Angle) + (180 - armJ2Angle) - 90), 0)));

        m_mechanism.update(0, armJ1Angle, armJ2Angle);

        fakeRobotPose.set(new Pose3d(2, 2, 0, new Rotation3d()));
    }

    public void resetMechanisms() {
        m_armSubsystem.resetArmPositions();
        m_elevatorSubsystem.resetMotorPosition();
    }

    public boolean checkMotorPositions() {
        return m_armSubsystem.checkMotorsSet() && m_elevatorSubsystem.checkMotorsSet();
    }

    public boolean isGoalReached() {
        return isGoalReached;
    }

    public ArmSubsystem getArmSubsystem() {
        return m_armSubsystem;
    }

    public ElevatorSubsystem getElevatorSubsystem() {
        return m_elevatorSubsystem;
    }

    public void periodic() {
        eleGeneralHeight = m_elevatorSubsystem.getEncoderDistance();
        armJ1Angle = m_armSubsystem.getFirstJointAngle();
        armJ2Angle = m_armSubsystem.getSecondJointAngle();
        updateMech();
        SmartDashboard.putString("MechState", lastState);
        SmartDashboard.putBoolean("MechGoalReached", isGoalReached);
    }
    

    public boolean isAlgaeGround()
    {
        return armJ1Angle < StatePositions.AlgaeGround[1] * 1.25 
                    && eleGeneralHeight > StatePositions.AlgaeGround[0] * 0.75;
    }

    public Command resetMechanismsCmd() {
        return new ParallelCommandGroup(m_armSubsystem.resetArmPositionsCmd(), m_elevatorSubsystem.resetMotorPositionCmd());
    }

    public Command resetEncodersCmd() {
        return new ParallelCommandGroup(m_armSubsystem.resetEncoderCmd(), m_elevatorSubsystem.resetEncoderCmd());
    }
}
