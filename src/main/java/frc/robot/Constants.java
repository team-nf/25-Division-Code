// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double MAX_SPEED = 5.0;

  public class Swerve
  {

    public static final double MaxSpeed = 0;
    public static final double MaxAngularRate = 0;

  }

  public class InitialConstants {
    public static final double[] EncoderStartAngles = {203.5/2,98/2};
    // **** Second joint should be equal to 355 at zeroing position
  }

  public static class TestingConstants {
    public static final boolean kTestWithSpinning = false;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double DEADBAND = 0.1;
  }

  public static class GripperConstants {
    public static final int kGripperID = 51;
    public static final double kGripper_kP = 0.11;
    public static final double kGripper_kI = 0.0;
    public static final double kGripper_kD = 0.0;
    public static final double kGripper_kS = 0.1;
    public static final double kGripper_kV = 0.12;
    public static final double kGripper_kPFV = 8.0;
    public static final double kGripper_LV = 8.0;
    public static final double kGripper_LA = 38;

    public static final int kAlgaeSensor = 9;
    public static final int kCoralSensor = 8;


  }

  public class Arm {

    public class FirstJoint {
      public static final int kMotorPort = 61;
      public static final int kEncoderChannel = 0;

      public static final String kArmPositionKey = "ArmPosition_J1";
      public static final String kArmPKey = "ArmP_J1";

      // The P gain for the PID controller that drives this arm.
      public static final double kArmJoint1_kP = 0.7;
      public static final double kArmJoint1_kI = 0.0;
      public static final double kArmJoint1_kD = 0.1;
      public static final double kArmJoint1_kS = 0.0;
      public static final double kArmJoint1_kV = 0.0;
      public static final double kArmJoint1_kA = 0.0;
      public static final double kArmJoint1_kG = 0.05;
      public static final double kArmJoint1_kPFV = 4;
      public static final double kArmJoint1_kSCL = 40;
      public static final double kArmJoint1_kSCLL = 15;

      public static final double kArmJoint1_MMCV = 280; // Cruise Velocity
      public static final double kArmJoint1_MMA = 250; // Acceleration
      public static final double kArmJoint1_MMJ = 0; // Jerk

      public static final double kDefaultArmSetpointDegrees = 75.0;

      // distance per pulse = (angle per revolution) / (pulses per revolution)
      //  = (2 * PI rads) / (4096 pulses)
      public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

      public static final double kArmReduction = 172.770;
      public static final double kArmMass = 6.5; // Kilograms
      public static final double kArmLength = 0.350;

      public static final double kMinAngle = 45;
      public static final double kMaxAngle = 250;

      public static final double kMinAngleRads = Units.degreesToRadians(-180);        // For sim
      public static final double kMaxAngleRads = Units.degreesToRadians(180); // For sim

      public static final double[] kSimOffsets = {0.091,0.056,0.275};

      public static final double kAngleTolerance = 4;
      public static final double kArmSafetyFactor = 1.5;

    }

   

    public class SecondJoint {
      public static final int kMotorPort = 62;
      public static final int kEncoderChannel = 1;
  

      public static final String kArmPositionKey = "ArmPosition_J2";
      public static final String kArmPKey = "ArmP_J2";

      // The P gain for the PID controller that drives this arm.
      public static final double kDefaultArmSetpointDegrees = 75.0;

      // The P gain for the PID controller that drives this arm.
      public static final double kArmJoint2_kP = 0.8;
      public static final double kArmJoint2_kI = 0.03;
      public static final double kArmJoint2_kD = 0.1;
      public static final double kArmJoint2_kS = 0.0;
      public static final double kArmJoint2_kV = 0.0;
      public static final double kArmJoint2_kA = 0.0;
      public static final double kArmJoint2_kG = 0.08;
      public static final double kArmJoint2_kPFV = 4;
      public static final double kArmJoint2_kSCL = 40;
      public static final double kArmJoint2_kSCLL = 15;

      public static final int kArmJoint2_MMCV = 150; // Cruise Velocity
      public static final int kArmJoint2_MMA = 100; // Acceleration
      public static final int kArmJoint2_MMJ = 0; // Jerk
      
      // distance per pulse = (angle per revolution) / (pulses per revolution)
      //  = (2 * PI rads) / (4096 pulses)
      public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

      public static final double kArmReduction = 84.69;
      public static final double kPulleyErrorRatio = 1/3;
      public static final double kArmMass = 4.0; // Kilograms
      public static final double kArmLength = 0.350;

      public static final double kMinAngle = 60;
      public static final double kMaxAngle = 250;

      public static final double kMinAngleRads = Units.degreesToRadians(-180);        // For sim
      public static final double kMaxAngleRads = Units.degreesToRadians(180); // For sim

      public static final double[] kSimOffsets = {0.091,0.007,0.275};
      
      public static final double kAngleTolerance = 4;
      public static final double kArmSafetyFactor = 1;
    }

  }
   

  public static class Elevator {
    public static final int kMotor1Port = 10;
    public static final int kMotor2Port = 11;

    public static final int kEncoderAChannel = 0;
    public static final int kEncoderBChannel = 1;

    public static final double kStage1Height = 0.65;
/** Benim denerken son ulaştığım değerleri geçiriyorum
    public static final double kElevatorKp = 0.5;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0.04;
    */
    public static final double kElevatorKp = 0.9;
    public static final double kElevatorKi = 0.04;
    public static final double kElevatorKd = 0.02;

    public static final double kElevatorkS = 0.0; // volts (V)
    //public static final double kElevatorkG = 0.8; // volts (V)
    public static final double kElevatorkG = 0.2; // ters yöne güç verince asansör yer çekimine karşı hareket ediyor değiştirmediysen bu kodda
    public static final double kElevatorkV = 0.0; // volt per velocity (V/(m/s))
    public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

    public static final double kElevatorMMCV = 400; //12V sınır koyunca maximum 90 cıvarına çıktı zaten velocity
    //+bence iki kat daha hızlı olabilir ama mümkün görünmüyor iyi bir sınır olmalı
    public static final double kElevatorMMA = 275;
    public static final double kElevatorMMJ = 0; // kullanmıyoruz

    public static final double kElevatorGearing = 56/12;
    public static final double kElevatorDrumRadius = 0.021;
    public static final double kElevatorDistPerRotation = 0.02;
    public static final double kCarriageMass = 13.0; // kg

    public static final double kAmpLimit = 38.0;
    public static final double kVoltageLimit = 10.0;

    public static final double kSetpointMeters = 0.75;
    // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
    public static final double kMinElevatorHeightMeters = 0.009;
    public static final double kMaxElevatorHeightMeters = 1.6;
    public static final double kElevatorTolerance = 0.03;
    public static final double kReadyPos = 0.04;
  }

  public class StatePositions
  {
    // Length, Angle 1, Angle 2
    
    public static final double angleOffset = -7;
    public static final double[] CoralIntake = {0.025, 212, 107};    //Coral Intake 217.5 110
    public static final double[] CoralStage1 = {0.05, 150, 160};      //Coral Stage 1
    public static final double[] CoralStage2 = {0.22, 150, 160};   //Coral Stage 2
    public static final double[] CoralStage3 = {0.63, 150, 160};    //Coral Stage 3
    public static final double[] CoralStage4 = {1.45, 150, 132};    //Coral Stage 4
    public static final double[] CoralCarry = {0.1, 170, 180};    //Coral Carry
    public static final double[] AlgaeThrowNet = {1.56, 180, 225};    //Algae Shoot
    public static final double[] AlgaeRecoverNet = {1.38, 190, 170};    //Algae Recover After Net
    public static final double[] AlgaeThrowProcessor = {0.09, 90, 230};    //Algae Shoot
    public static final double[] AlgaeStage23 = {0.40, 120, 233};     //Algae Stage 2-3
    public static final double[] AlgaeStage34 = {0.90, 120, 233};     //Algae Stage 3-4
    public static final double[] AlgaeGround = {0.28, 45, 257.5};  //Algae Ground
    public static final double[] AlgaeFromCoral = {0.25, 180, 180};  //Algae Ground
    public static final double[] AlgaeCarry = {0.3, 150, 210};  //Algae Ground
    public static final double[] Closed = {0.5, 180, 180};  //Closed
    public static final double[] FullyClosed = {0.01, 180, 180};

  }

  public class AutoConstants
  {
    public static final double[] stage4Pos18R   = {3.15,3.95,0};
    public static final double[] stage3Pos18R   = {3.15,3.95,0};
    public static final double[] stage4Pos18L   = {3.15,4.32,0};
    public static final double[] stage3Pos18L   = {3.15,4.32,0};
    public static final double[] algae3Pos      = {2.84,3.8,0};
    public static final double[] algae2Pos      = {2.84,3.8,0};
    public static final double[] checkTagPos18R = {2.85,3.95,0};
    public static final double[] checkTagPos18L = {2.85,4.32,0};
    public static final double[] checkTagPos18  = {2.6,4.1,0};


    public static final double[] algaeNetPos14       = {1.5,6,0};
    public static final double[] algaeProcessorPos16 = {2.98,4,0};
    public static final double[] intakeCoralPos13    = {1.047,6.962,-54};
    //public static final double[] intakeCoralPos13    = {1.19,7.24,-56};

    public static final Pose2d stage4RPose2D  = new Pose2d(stage4Pos18R[0], stage4Pos18R[1], Rotation2d.fromDegrees(stage4Pos18R[2]));
    public static final Pose2d stage3RPose2D  = new Pose2d(stage3Pos18R[0], stage3Pos18R[1], Rotation2d.fromDegrees(stage3Pos18R[2]));
    public static final Pose2d stage4LPose2D  = new Pose2d(stage4Pos18L[0], stage4Pos18L[1], Rotation2d.fromDegrees(stage4Pos18L[2]));
    public static final Pose2d stage3LPose2D  = new Pose2d(stage3Pos18L[0], stage3Pos18L[1], Rotation2d.fromDegrees(stage3Pos18L[2]));
    public static final Pose2d algae3Pose2D   = new Pose2d(algae3Pos[0], algae3Pos[1], Rotation2d.fromDegrees(algae3Pos[2]));
    public static final Pose2d algae2Pose2D   = new Pose2d(algae2Pos[0], algae2Pos[1], Rotation2d.fromDegrees(algae2Pos[2]));
    public static final Pose2d checkTagRPose2D = new Pose2d(checkTagPos18R[0], checkTagPos18R[1], Rotation2d.fromDegrees(checkTagPos18R[2]));
    public static final Pose2d checkTagLPose2D = new Pose2d(checkTagPos18L[0], checkTagPos18L[1], Rotation2d.fromDegrees(checkTagPos18L[2]));
    public static final Pose2d checkTagPose2D  = new Pose2d(checkTagPos18[0], checkTagPos18[1], Rotation2d.fromDegrees(checkTagPos18[2]));

    public static final Pose2d algaeNet14Pose2D       = new Pose2d(algaeNetPos14[0], algaeNetPos14[1], Rotation2d.fromDegrees(algaeNetPos14[2]));
    public static final Pose2d algaeProcessor16Pose2D = new Pose2d(algaeProcessorPos16[0], algaeProcessorPos16[1], Rotation2d.fromDegrees(algaeProcessorPos16[2]));
    public static final Pose2d intakeCoral13Pose2D    = new Pose2d(intakeCoralPos13[0], intakeCoralPos13[1], Rotation2d.fromDegrees(intakeCoralPos13[2]));

    public static final double[] tag1 = {16.6972, 0.6553, 1.4859, 126, 0};
    public static final double[] tag2 = {16.6972, 7.3965, 1.4859, 234, 0};
    public static final double[] tag3 = {11.5608, 8.0556, 1.3017, 270, 0};
    public static final double[] tag4 = {9.2761, 6.1377, 1.8679, 0, 30};
    public static final double[] tag5 = {9.2761, 1.9149, 1.8679, 0, 30};
    public static final double[] tag6 = {13.4744, 3.3063, 0.3081, 300, 0};
    public static final double[] tag7 = {13.8905, 4.0259, 0.3081, 0, 0};
    public static final double[] tag8 = {13.4744, 4.7455, 0.3081, 60, 0};
    public static final double[] tag9 = {12.6434, 4.7455, 0.3081, 120, 0};
    public static final double[] tag10 = {12.2273, 4.0259, 0.3081, 180, 0};
    public static final double[] tag11 = {12.6434, 3.3063, 0.3081, 240, 0};
    public static final double[] tag12 = {0.8512, 0.6553, 1.4859, 54, 0};
    public static final double[] tag13 = {0.8512, 7.3965, 1.4859, 306, 0};
    public static final double[] tag14 = {8.2723, 6.1377, 1.8679, 180, 30};
    public static final double[] tag15 = {8.2723, 1.9149, 1.8679, 180, 30};
    public static final double[] tag16 = {5.9875, -0.0038, 1.3017, 90, 0};
    public static final double[] tag17 = {4.0739, 3.3063, 0.3081, 240, 0};
    public static final double[] tag18 = {3.6576, 4.0259, 0.3081, 180, 0};
    public static final double[] tag19 = {4.0739, 4.7455, 0.3081, 120, 0};
    public static final double[] tag20 = {4.9047, 4.7455, 0.3081, 60, 0};
    public static final double[] tag21 = {5.3210, 4.0259, 0.3081, 0, 0};
    public static final double[] tag22 = {4.9047, 3.3063, 0.3081, 300, 0};

    public static final Pose2d TAG18_POSE2D = new Pose2d(tag18[0], tag18[1], Rotation2d.fromDegrees(tag18[3]));
    public static final Pose2d TAG16_POSE2D = new Pose2d(tag16[0], tag16[1], Rotation2d.fromDegrees(tag16[3]));
    public static final Pose2d TAG14_POSE2D = new Pose2d(tag14[0], tag14[1], Rotation2d.fromDegrees(tag14[3]));
    public static final Pose2d TAG13_POSE2D = new Pose2d(tag13[0], tag13[1], Rotation2d.fromDegrees(tag13[3]));

    public static final Transform2d RobotPosByTagR = new Transform2d(TAG18_POSE2D, checkTagRPose2D);
    public static final Transform2d RobotPosByTagL = new Transform2d(TAG18_POSE2D, checkTagLPose2D);
    public static final Transform2d RobotPosByTag = new Transform2d(TAG18_POSE2D, checkTagPose2D);
    public static final Transform2d ReefPosS4RByTag =  new Transform2d(TAG18_POSE2D, stage4RPose2D);
    public static final Transform2d ReefPosS3RByTag =  new Transform2d(TAG18_POSE2D, stage3RPose2D);
    public static final Transform2d ReefPosS4LByTag =  new Transform2d(TAG18_POSE2D, stage4LPose2D);
    public static final Transform2d ReefPosS3LByTag =  new Transform2d(TAG18_POSE2D, stage3LPose2D);
    public static final Transform2d Algae3ByTag =  new Transform2d(TAG18_POSE2D, algae3Pose2D);
    public static final Transform2d Algae2ByTag =  new Transform2d(TAG18_POSE2D, algae2Pose2D);

    public static final Transform2d AlgaeNetByTag =  new Transform2d(TAG14_POSE2D, algaeNet14Pose2D);
    public static final Transform2d AlgaeProcessorByTag =  new Transform2d(TAG16_POSE2D, algaeProcessor16Pose2D);
    public static final Transform2d IntakeCoralByTag =  new Transform2d(TAG13_POSE2D, intakeCoral13Pose2D);

    public static final double LL_Accuracy_mt1 = 0.03;
  }

}
