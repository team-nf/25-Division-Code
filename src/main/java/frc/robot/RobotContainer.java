// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Algae23Cmd;
import frc.robot.commands.Algae34Cmd;
import frc.robot.commands.AlgaeCarryCmd;
import frc.robot.commands.AlgaeGroundCmd;
import frc.robot.commands.AlgaeNetCmd;
import frc.robot.commands.AlgaeProCmd;
import frc.robot.commands.AlgaeTrackCmd;
import frc.robot.commands.ClosedCmd;
import frc.robot.commands.ClosedFullyCmd;
import frc.robot.commands.CoralCarryCmd;
import frc.robot.commands.CoralIntakeCmd;
import frc.robot.commands.L1Cmd;
import frc.robot.commands.L2Cmd;
import frc.robot.commands.L3Cmd;
import frc.robot.commands.L4Cmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final GripperSubsystem m_gripperSubsystem = new GripperSubsystem();

  private final MainMechStateMachine m_mainMech = new MainMechStateMachine(m_armSubsystem, m_elevatorSubsystem, m_gripperSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_operatorController =
      new CommandPS4Controller(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_driverController =
      new CommandXboxController(1);

  private final SendableChooser<Integer> m_reefChooser = new SendableChooser<>();
  private final SendableChooser<Boolean> m_isColorBlue = new SendableChooser<>();
  private final SendableChooser<Boolean> m_isAlgaeMode = new SendableChooser<>();
  private final SendableChooser<Integer> m_checkIntake = new SendableChooser<>();

  public final CommandSwerveDrivetrain m_swerve = TunerConstants.createDrivetrain();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(m_swerve.getMaxSpeed() * 0.05).withRotationalDeadband(m_swerve.getMaxAngularRate() * 0.015) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
    .withDeadband(m_swerve.getMaxSpeed() * 0.05).withRotationalDeadband(m_swerve.getMaxAngularRate() * 0.015) // Add a 10% deadband5
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 


  private final double kAngle = 0.3;
  private final double kDrive = 0.8;

  private double selectedReef = -1;
  private boolean isBlueSelected = true;
  private boolean isAlgaeSelected = false;
  private double selectedReefID = -1;

  private SlewRateLimiter d1Filter = new SlewRateLimiter(2.8);
  private SlewRateLimiter d2Filter = new SlewRateLimiter(2.8);
  private SlewRateLimiter rFilter = new SlewRateLimiter(3);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    NamedCommands.registerCommand("Closed",         new ClosedCmd(m_mainMech));
    NamedCommands.registerCommand("FullyClosed",    new ClosedFullyCmd(m_mainMech));
    NamedCommands.registerCommand("CoralIntake",    new CoralIntakeCmd(m_mainMech));
    NamedCommands.registerCommand("CoralCarry",     new CoralCarryCmd(m_mainMech));
    NamedCommands.registerCommand("CoralStage1",    new L1Cmd(m_mainMech));
    NamedCommands.registerCommand("CoralStage2",    new L2Cmd(m_mainMech));
    NamedCommands.registerCommand("CoralStage3",    new L3Cmd(m_mainMech));
    NamedCommands.registerCommand("CoralStage4",    new L4Cmd(m_mainMech));
    NamedCommands.registerCommand("Algae23",        new Algae23Cmd(m_mainMech));
    NamedCommands.registerCommand("Algae34",        new Algae34Cmd(m_mainMech));
    NamedCommands.registerCommand("AlgaeCarry",     new AlgaeCarryCmd(m_mainMech));
    NamedCommands.registerCommand("AlgaeNet",       new AlgaeNetCmd(m_mainMech));
    NamedCommands.registerCommand("AlgaeProcessor", new AlgaeProCmd(m_mainMech));
    NamedCommands.registerCommand("AlgaeGround",    new AlgaeGroundCmd(m_mainMech));
    NamedCommands.registerCommand("AlgaeTrack",     new AlgaeTrackCmd(m_swerve));



    NamedCommands.registerCommand("TakeCoralAuto", m_gripperSubsystem.TakeCoralAutoCommand());
    NamedCommands.registerCommand("ThrowCoralAuto", m_gripperSubsystem.ThrowCoralAutoCommand());


    configureBindings();


    m_swerve.setDefaultCommand(
      // Drivetrain will execute this command periodically
      m_swerve.applyRequest(() ->
          drive.withVelocityX(-d1Filter.calculate(m_driverController.getLeftY()) * m_swerve.getMaxSpeed() * m_swerve.getDriveMultiplier()* kDrive) // Drive forward with negative Y (forward)
              .withVelocityY(-d2Filter.calculate(m_driverController.getLeftX()) * m_swerve.getMaxSpeed() * m_swerve.getDriveMultiplier() * kDrive) // Drive left with negative X (left)
              .withRotationalRate(-rFilter.calculate(m_driverController.getRightX()) * m_swerve.getMaxAngularRate() * kAngle) // Drive counterclockwise with negative X (left)
      ));

    m_reefChooser.setDefaultOption("1", 1);
    m_reefChooser.addOption("2", 2);
    m_reefChooser.addOption("3", 3);
    m_reefChooser.addOption("4", 4);
    m_reefChooser.addOption("5", 5);
    m_reefChooser.addOption("6", 6);

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
          m_isColorBlue.setDefaultOption("Red", false);
          m_isColorBlue.addOption("Blue", true);
        }

        if (ally.get() == Alliance.Blue) {
          m_isColorBlue.setDefaultOption("Blue", true);
          m_isColorBlue.addOption("red", false);

        }
    }
    else {
      m_isColorBlue.setDefaultOption("Blue", true);
      m_isColorBlue.addOption("Red", false);
    }

    m_isAlgaeMode.setDefaultOption("Coral", false);
    m_isAlgaeMode.addOption("Algae", true);

    m_checkIntake.setDefaultOption("1", 1);
    m_checkIntake.setDefaultOption("2", 2);
    m_checkIntake.setDefaultOption("12", 12);
    m_checkIntake.setDefaultOption("13", 13);

    SmartDashboard.putData("ReefN", m_reefChooser);
    SmartDashboard.putData("ColorSelect", m_isColorBlue);
    SmartDashboard.putData("AlgaeMode", m_isAlgaeMode);
    SmartDashboard.putData("IntakeSelect", m_checkIntake);

    m_mainMech.resetMechanisms();
  }
  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    m_operatorController.cross().whileTrue(NamedCommands.getCommand("CoralStage1"));
    m_operatorController.circle().whileTrue(NamedCommands.getCommand("CoralStage2"));
    m_operatorController.square().whileTrue(NamedCommands.getCommand("CoralStage3"));
    m_operatorController.triangle().whileTrue(NamedCommands.getCommand("CoralStage4"));
    m_operatorController.L1().whileTrue(NamedCommands.getCommand("FullyClosed"));
    m_operatorController.R1().whileTrue(NamedCommands.getCommand("AlgaeNet"));
    m_operatorController.L2().whileTrue(NamedCommands.getCommand("CoralIntake"));
    m_operatorController.R2().whileTrue(
        new ParallelCommandGroup(
            NamedCommands.getCommand("AlgaeGround"),
            NamedCommands.getCommand("AlgaeTrack")
        )
    );
    m_operatorController.L3().whileTrue(NamedCommands.getCommand("CoralCarry"));
    m_operatorController.R3().whileTrue(NamedCommands.getCommand("AlgaeCarry"));
    m_operatorController.share().whileTrue(NamedCommands.getCommand("Algae23"));
    m_operatorController.options().whileTrue(NamedCommands.getCommand("Algae34"));

    m_driverController.rightBumper().whileTrue(m_swerve.applyRequest(() ->
    driveRobotCentric.withVelocityX(-m_driverController.getLeftY() * m_swerve.getMaxSpeed() * m_swerve.getDriveMultiplier()* kDrive * 0.3) // Drive forward with negative Y (forward)
        .withVelocityY(-m_driverController.getLeftX() * m_swerve.getMaxSpeed() * m_swerve.getDriveMultiplier() * kDrive * 0.3) // Drive left with negative X (left)
        .withRotationalRate(-m_driverController.getRightX() * m_swerve.getMaxAngularRate() * kAngle) // Drive counterclockwise with negative X (left)
    ));

    m_driverController.pov(0).onTrue(m_gripperSubsystem.takeAlgae());
    m_driverController.pov(90).onTrue(m_gripperSubsystem.takeCoral());
    m_driverController.pov(180).whileTrue(m_gripperSubsystem.throwAlgae().andThen(m_swerve.getOut())); //.andThen(NamedCommands.getCommand("FullyClosed"))
    m_driverController.pov(270).whileTrue(m_gripperSubsystem.throwCoral().andThen(m_swerve.getOut()));

    m_driverController.back().whileTrue(NamedCommands.getCommand("CoralIntake"));
    m_driverController.start().onTrue(m_swerve.resetHeading());
    m_driverController.rightStick().whileTrue(NamedCommands.getCommand("FullyClosed"));

    //m_driverController.button(4).onTrue(m_swerve.setPose(0.5,2.0,0));

    m_driverController.leftBumper().and(()-> {return checkIntake(1);})
      .whileTrue(NamedCommands.getCommand("CoralIntake").withDeadline(m_swerve.goToIntake(1)).andThen(NamedCommands.getCommand("TakeCoralAuto")));
    m_driverController.leftBumper().and(()-> {return checkIntake(2);})
      .whileTrue(NamedCommands.getCommand("CoralIntake").withDeadline(m_swerve.goToIntake(2)).andThen(NamedCommands.getCommand("TakeCoralAuto")));
    m_driverController.leftBumper().and(()-> {return checkIntake(12);})
      .whileTrue(NamedCommands.getCommand("CoralIntake").withDeadline(m_swerve.goToIntake(12)).andThen(NamedCommands.getCommand("TakeCoralAuto")));
    m_driverController.leftBumper().and(()-> {return checkIntake(13);})
      .whileTrue(NamedCommands.getCommand("CoralIntake").withDeadline(m_swerve.goToIntake(13)).andThen(NamedCommands.getCommand("TakeCoralAuto")));

    //CORAL MODE -->

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(1, true);})
      .whileTrue(autoReefPoseS4R(17));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(1, true);})
      .whileTrue(autoReefPoseS3L(17));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(1, true);})
      .whileTrue(autoReefPoseS4L(17));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(1, true);})
      .whileTrue(autoReefPoseS3R(17));
    m_driverController.leftTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(1, true);})
      .whileTrue(autoReefPoseS2L(17));
    m_driverController.rightTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(1, true);})
      .whileTrue(autoReefPoseS2R(17));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(2, true);})
      .whileTrue(autoReefPoseS4R(18));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(2, true);})
      .whileTrue(autoReefPoseS3L(18));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(2, true);})
      .whileTrue(autoReefPoseS4L(18));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(2, true);})
      .whileTrue(autoReefPoseS3R(18));
    m_driverController.leftTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(2, true);})
      .whileTrue(autoReefPoseS2L(18));
    m_driverController.rightTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(2, true);})
      .whileTrue(autoReefPoseS2R(18));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(3, true);})
      .whileTrue(autoReefPoseS4R(19));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(3, true);})
      .whileTrue(autoReefPoseS3L(19));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(3, true);})
      .whileTrue(autoReefPoseS4L(19));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(3, true);})
      .whileTrue(autoReefPoseS3R(19));
    m_driverController.leftTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(3, true);})
      .whileTrue(autoReefPoseS2L(19));
    m_driverController.rightTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(3, true);})
      .whileTrue(autoReefPoseS2R(19));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(4, true);})
      .whileTrue(autoReefPoseS4R(20));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(4, true);})
      .whileTrue(autoReefPoseS3L(20));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(4, true);})
      .whileTrue(autoReefPoseS4L(20));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(4, true);})
      .whileTrue(autoReefPoseS3R(20));
    m_driverController.leftTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(4, true);})
      .whileTrue(autoReefPoseS2L(20));
    m_driverController.rightTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(4, true);})
      .whileTrue(autoReefPoseS2R(20));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(5, true);})
      .whileTrue(autoReefPoseS4R(21));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(5, true);})
      .whileTrue(autoReefPoseS3L(21));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(5, true);})
      .whileTrue(autoReefPoseS4L(21));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(5, true);})
      .whileTrue(autoReefPoseS3R(21));
    m_driverController.leftTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(5, true);})
      .whileTrue(autoReefPoseS2L(21));
    m_driverController.rightTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(5, true);})
      .whileTrue(autoReefPoseS2R(21));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(6, true);})
      .whileTrue(autoReefPoseS4R(22));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(6, true);})
      .whileTrue(autoReefPoseS3L(22));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(6, true);})
      .whileTrue(autoReefPoseS4L(22));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(6, true);})
      .whileTrue(autoReefPoseS3R(22));
    m_driverController.leftTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(6, true);})
      .whileTrue(autoReefPoseS2L(22));
    m_driverController.rightTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(6, true);})
      .whileTrue(autoReefPoseS2R(22));
   
    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(1, false);})
      .whileTrue(autoReefPoseS4R(8));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(1, false);})
      .whileTrue(autoReefPoseS3L(8));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(1, false);})
      .whileTrue(autoReefPoseS4L(8));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(1, false);})
      .whileTrue(autoReefPoseS3R(8));
    m_driverController.leftTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(1, false);})
      .whileTrue(autoReefPoseS2L(8));
    m_driverController.rightTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(1, false);})
      .whileTrue(autoReefPoseS2R(8));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(2, false);})
      .whileTrue(autoReefPoseS4R(7));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(2, false);})
      .whileTrue(autoReefPoseS3L(7));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(2, false);})
      .whileTrue(autoReefPoseS4L(7));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(2, false);})
      .whileTrue(autoReefPoseS3R(7));
    m_driverController.leftTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(2, false);})
      .whileTrue(autoReefPoseS2L(7));
    m_driverController.rightTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(2, false);})
      .whileTrue(autoReefPoseS2R(7));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(3, false);})
      .whileTrue(autoReefPoseS4R(6));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(3, false);})
      .whileTrue(autoReefPoseS3L(6));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(3, false);})
      .whileTrue(autoReefPoseS4L(6));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(3, false);})
      .whileTrue(autoReefPoseS3R(6));
    m_driverController.leftTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(3, false);})
      .whileTrue(autoReefPoseS2L(6));
    m_driverController.rightTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(3, false);})
      .whileTrue(autoReefPoseS2R(6));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(4, false);})
      .whileTrue(autoReefPoseS4R(11));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(4, false);})
      .whileTrue(autoReefPoseS3L(11));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(4, false);})
      .whileTrue(autoReefPoseS4L(11));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(4, false);})
      .whileTrue(autoReefPoseS3R(11));
    m_driverController.leftTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(4, false);})
      .whileTrue(autoReefPoseS2L(11));
    m_driverController.rightTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(4, false);})
      .whileTrue(autoReefPoseS2R(11));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(5, false);})
      .whileTrue(autoReefPoseS4R(10));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(5, false);})
      .whileTrue(autoReefPoseS3L(10));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(5, false);})
      .whileTrue(autoReefPoseS4L(10));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(5, false);})
      .whileTrue(autoReefPoseS3R(10));
    m_driverController.leftTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(5, false);})
      .whileTrue(autoReefPoseS2L(10));
    m_driverController.rightTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(5, false);})
      .whileTrue(autoReefPoseS2R(10));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(6, false);})
      .whileTrue(autoReefPoseS4R(9));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(6, false);})
      .whileTrue(autoReefPoseS3L(9));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(6, false);})
      .whileTrue(autoReefPoseS4L(9));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(6, false);})
      .whileTrue(autoReefPoseS3R(9));
    m_driverController.leftTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(6, false);})
      .whileTrue(autoReefPoseS2L(9));
    m_driverController.rightTrigger().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(6, false);})
      .whileTrue(autoReefPoseS2R(9));

    // CORAL MODE <--> ALGAE MODE

    m_driverController.a().and(() -> {return isAlgaeSelected;}).whileTrue(
        NamedCommands.getCommand("AlgaeGround")
        .andThen(NamedCommands.getCommand("AlgaeTrack"))
    );
    m_driverController.b().and(() -> {return isAlgaeSelected;}).whileTrue(NamedCommands.getCommand("AlgaeCarry"));

    m_driverController.x().and(() -> {return isAlgaeSelected;}).and(() -> {return checkTeam(true);}).whileTrue(m_swerve.goToBlueNet());
    m_driverController.x().and(() -> {return isAlgaeSelected;}).and(() -> {return checkTeam(false);}).whileTrue(m_swerve.goToRedNet());

    m_driverController.rightTrigger().and(() -> {return isAlgaeSelected;}).whileTrue(NamedCommands.getCommand("AlgaeNet"));
    m_driverController.leftTrigger().and(() -> {return isAlgaeSelected;}).whileTrue(NamedCommands.getCommand("AlgaeProcessor"));

    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(1, true);})
        .whileTrue(autoReefA23(17));
    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(2, true);})
        .whileTrue(autoReefA34(18));
    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(3, true);})
        .whileTrue(autoReefA23(19));
    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(4, true);})
        .whileTrue(autoReefA34(20));
    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(5, true);})
        .whileTrue(autoReefA23(21));  
    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(6, true);})
        .whileTrue(autoReefA34(22));


    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(1, false);})
        .whileTrue(autoReefA23(8));
    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(2, false);})
        .whileTrue(autoReefA34(7));
    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(3, false);})
        .whileTrue(autoReefA23(6));
    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(4, false);})
        .whileTrue(autoReefA34(11));
    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(5, false);})
        .whileTrue(autoReefA23(10));      
    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(6, false);})
        .whileTrue(autoReefA34(9));


    // ALGAE MODE <--

    /*

    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(1, false);}).whileTrue(m_swerve.goToAlgae(8, 3)
      .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(1, false);}).whileTrue(m_swerve.goToAlgae(8, 2)
      .andThen(NamedCommands.getCommand("Algae23")));

    */
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public boolean checkReef(int reefTag, boolean isBlue) {
    if(isBlue) reefTag += 16;
    else
    {
      reefTag = 9 - reefTag;
      if(reefTag <= 5) reefTag += 6;
    }
    return ((selectedReefID == reefTag) && (isBlueSelected == isBlue));
  }

  public boolean checkTeam(boolean isBlue) {
    return isBlueSelected == isBlue;
  }

  public void resetEncoders()
  {
    m_armSubsystem.resetEncoders();
  }

  public void putSelectedReefID() {
    SmartDashboard.putNumber("SelectedReef",  selectedReef);
    SmartDashboard.putNumber("SelectedReefID",  selectedReefID);
    SmartDashboard.putBoolean("IsBlueSelected", isBlueSelected);
    SmartDashboard.putBoolean("IsAlgaeSelected", isAlgaeSelected);
  }


  public Command getAutonomousCommandBlue() { 
    // An example command will be run in autonomous
    return m_swerve.setPoseBlueAuto()
                                     .andThen(NamedCommands.getCommand("CoralCarry").withDeadline(m_swerve.goToTagAuto(20)))
                                     .andThen(new ParallelCommandGroup(m_swerve.goToReef(20, true, 4), (NamedCommands.getCommand("CoralStage4").withTimeout(4.5))))
                                     .andThen(NamedCommands.getCommand("ThrowCoralAuto"))
                                     .andThen(new ParallelCommandGroup(NamedCommands.getCommand("CoralIntake").withDeadline(m_swerve.goToIntakeAuto(13)),NamedCommands.getCommand("TakeCoralAuto")))
                                     .andThen(NamedCommands.getCommand("CoralCarry").withDeadline(m_swerve.goToTagAuto(19)))
                                     .andThen(new ParallelCommandGroup(m_swerve.goToReef(19, false, 4), (NamedCommands.getCommand("CoralStage4").withTimeout(4.5))))
                                     .andThen(NamedCommands.getCommand("CoralStage4"))
                                     .andThen(NamedCommands.getCommand("ThrowCoralAuto"))
                                     .andThen(NamedCommands.getCommand("CoralIntake").withDeadline(m_swerve.goToTag(19)));
  }

  public Command getAutonomousCommandRed() { 
    // An example command will be run in autonomous
    return m_swerve.setPoseRedAuto()
                                     .andThen(NamedCommands.getCommand("CoralCarry").withDeadline(m_swerve.goToTagAuto(11)))
                                     .andThen(new ParallelCommandGroup(m_swerve.goToReef(11, true, 4), (NamedCommands.getCommand("CoralStage4").withTimeout(4.5))))
                                     .andThen(NamedCommands.getCommand("ThrowCoralAuto"))
                                     .andThen(new ParallelCommandGroup(NamedCommands.getCommand("CoralIntake").withDeadline(m_swerve.goToIntakeAuto(1)),NamedCommands.getCommand("TakeCoralAuto")))
                                     .andThen(NamedCommands.getCommand("CoralCarry").withDeadline(m_swerve.goToTagAuto(6)))
                                     .andThen(new ParallelCommandGroup(m_swerve.goToReef(6, false, 4), (NamedCommands.getCommand("CoralStage4").withTimeout(4.5))))
                                     .andThen(NamedCommands.getCommand("CoralStage4"))
                                     .andThen(NamedCommands.getCommand("ThrowCoralAuto"))
                                     .andThen(NamedCommands.getCommand("CoralIntake").withDeadline(m_swerve.goToTag(6)));
  }

  public Command getAutonomousCommand() { 
    // An example command will be run in autonomous
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Blue) {
          return getAutonomousCommandBlue();
        }
        else return getAutonomousCommandRed();
      }
    return null;
  }

  public Command getAutoTest()
  {
    return null;
  }

  public void setSelectorInfos()
  {
    isAlgaeSelected = m_isAlgaeMode.getSelected().booleanValue();
    isBlueSelected = m_isColorBlue.getSelected().booleanValue();
    selectedReef = m_reefChooser.getSelected().intValue();
    selectedReefID = selectedReef;
    if(isBlueSelected) selectedReefID += 16;
    else
    {
      selectedReefID = 9 - selectedReefID;
      if(selectedReefID <= 5) selectedReefID += 6;
    }
  }

  public boolean checkIntake(int n)
  {
    return n == m_checkIntake.getSelected();
  }

  public void containerPeriodic()
  {
    putSelectedReefID();
    setSelectorInfos();
    m_mainMech.periodic();
  }

  public Command autoReefPoseS4L(int id)
  {
    return m_swerve.goToTagL(id).andThen(NamedCommands.getCommand("CoralStage4").withDeadline(m_swerve.goToReefWithPID(id, true, 4)));
  }

  public Command autoReefPoseS4R(int id)
  {
    return m_swerve.goToTagR(id).andThen(NamedCommands.getCommand("CoralStage4").withDeadline(m_swerve.goToReefWithPID(id, false, 4)));
  }

  public Command autoReefPoseS3L(int id)
  {
    return m_swerve.goToTagL(id).andThen(NamedCommands.getCommand("CoralStage3").withDeadline(m_swerve.goToReefWithPID(id, true, 3)));
  }

  public Command autoReefPoseS3R(int id)
  {
    return m_swerve.goToTagR(id).andThen(NamedCommands.getCommand("CoralStage3").withDeadline(m_swerve.goToReefWithPID(id, false, 3)));
  }

  
  public Command autoReefPoseS2L(int id)
  {
    return m_swerve.goToTagL(id).andThen(NamedCommands.getCommand("CoralStage2").withDeadline(m_swerve.goToReefWithPID(id, true, 3)));
  }

  public Command autoReefPoseS2R(int id)
  {
    return m_swerve.goToTagR(id).andThen(NamedCommands.getCommand("CoralStage2").withDeadline(m_swerve.goToReefWithPID(id, false, 3)));
  }

  public Command autoReefPoseS1L(int id)
  {
    return m_swerve.goToTagL(id).andThen(NamedCommands.getCommand("CoralStage1").withDeadline(m_swerve.goToReefWithPID(id, true, 3)));
  }

  public Command autoReefPoseS1R(int id)
  {
    return m_swerve.goToTagR(id).andThen(NamedCommands.getCommand("CoralStage1").withDeadline(m_swerve.goToReefWithPID(id, false, 3)));
  }

  public Command autoReefA23(int id)
  {
    return m_swerve.goToAlgae(id,2).andThen(NamedCommands.getCommand("Algae23"));
  }

  public Command autoReefA34(int id)
  {
    return m_swerve.goToAlgae(id,3).andThen(NamedCommands.getCommand("Algae34"));
  }



}
