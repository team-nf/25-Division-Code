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

  private final SwerveRequest.RobotCentric drivRobotCentric = new SwerveRequest.RobotCentric()
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
    NamedCommands.registerCommand("Closed",         new ClosedCmd(m_mainMech));
    NamedCommands.registerCommand("FullyClosed",    new ClosedFullyCmd(m_mainMech));


    //NamedCommands.registerCommand("TakeCoralAuto", m_gripperSubsystem.TakeCoralAutoCommand());
    //NamedCommands.registerCommand("ThrowCoralAuto", m_gripperSubsystem.ThrowCoralAutoCommand());


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
    /* 
    m_operatorController.square().onTrue(NamedCommands.getCommand("Algae23"));
    m_operatorController.circle().onTrue(NamedCommands.getCommand("Algae34"));
    m_operatorController.cross().onTrue(NamedCommands.getCommand("AlgaeGround"));
    m_operatorController.triangle().onTrue(NamedCommands.getCommand("AlgaeCarry"));
    m_operatorController.L1().onTrue(NamedCommands.getCommand("AlgaeProcessor"));
    m_operatorController.R1().onTrue(NamedCommands.getCommand("AlgaeNet"));
    */
    //m_operatorController.button(9).onTrue(NamedCommands.getCommand("TestCommand"));
   // m_operatorController.button(9).onTrue(m_gripperSubsystem.stopCommand());
    /*
    m_driverController.pov(0).onTrue(m_gripperSubsystem.takeAlgae());
    m_driverController.pov(90).whileTrue(m_gripperSubsystem.throwAlgae());
    m_driverController.pov(180).onTrue(m_gripperSubsystem.takeCoral());
    m_driverController.pov(270).whileTrue(m_gripperSubsystem.throwCoral());

    m_driverController.button(8).onTrue(NamedCommands.getCommand("FullyClosed"));
    m_driverController.button(10).onTrue(NamedCommands.getCommand("Closed"));

    m_driverController.button(5).onTrue(NamedCommands.getCommand("CoralIntake").andThen(m_gripperSubsystem.takeCoral()));

    m_driverController.button(7).and(() -> checkTeam(true)).whileTrue(m_swerve.goToBlueNet());
    m_driverController.button(7).and(() -> checkTeam(false)).whileTrue(m_swerve.goToRedNet());
    */
    m_driverController.button(6).onTrue(m_swerve.resetHeading());
    m_driverController.rightTrigger(0.5).whileTrue(m_swerve.applyRequest(() ->
    drivRobotCentric.withVelocityX(m_driverController.getLeftY() * m_swerve.getMaxSpeed() * m_swerve.getDriveMultiplier()* kDrive * 0.3) // Drive forward with negative Y (forward)
        .withVelocityY(-m_driverController.getLeftX() * m_swerve.getMaxSpeed() * m_swerve.getDriveMultiplier() * kDrive * 0.3) // Drive left with negative X (left)
        .withRotationalRate(-m_driverController.getRightX() * m_swerve.getMaxAngularRate() * kAngle) // Drive counterclockwise with negative X (left)
    ));

    //m_operatorController.pov(0).whileTrue(m_gripperSubsystem.runGripper(0.5));
    //m_operatorController.pov(90).whileTrue(m_gripperSubsystem.runGripper(0.3));
    //m_operatorController.pov(180).whileTrue(m_gripperSubsystem.runGripper(-0.5));
    //m_operatorController.pov(270).whileTrue(m_gripperSubsystem.runGripper(-0.3));

    m_operatorController.cross().whileTrue(NamedCommands.getCommand("CoralStage1"));
    m_operatorController.circle().whileTrue(NamedCommands.getCommand("CoralStage2"));
    m_operatorController.square().whileTrue(NamedCommands.getCommand("CoralStage3"));
    m_operatorController.triangle().whileTrue(NamedCommands.getCommand("CoralStage4"));
    m_operatorController.R1().onTrue(NamedCommands.getCommand("AlgaeNet"));


    m_driverController.a().whileTrue(m_swerve.goToReefWithPath(6, false, 4));
    m_driverController.b().whileTrue(m_swerve.goToTag(6).andThen( new ParallelCommandGroup(m_swerve.goToReef(6, false, 4),NamedCommands.getCommand("CoralStage4"))));
    m_driverController.x().whileTrue(m_swerve.goToTag(6).andThen(m_swerve.goToReefWithPID(6, false, 4)));
    m_driverController.y().whileTrue(m_swerve.goToTag(6).andThen(NamedCommands.getCommand("CoralStage4").withDeadline(m_swerve.goToReefWithPID(6, false, 4))));

    
    //m_driverController.button(4).onTrue(m_swerve.setPose(0.5,2.0,0));


    /* 
    m_driverController.leftTrigger(0.5).and(()-> {return checkIntake(1);})
      .whileTrue(NamedCommands.getCommand("CoralIntake").withDeadline(m_swerve.goToIntake(1)).andThen(NamedCommands.getCommand("TakeCoralAuto")));
    m_driverController.leftTrigger(0.5).and(()-> {return checkIntake(2);})
      .whileTrue(NamedCommands.getCommand("CoralIntake").withDeadline(m_swerve.goToIntake(2)).andThen(NamedCommands.getCommand("TakeCoralAuto")));
    m_driverController.leftTrigger(0.5).and(()-> {return checkIntake(12);})
      .whileTrue(NamedCommands.getCommand("CoralIntake").withDeadline(m_swerve.goToIntake(12)).andThen(NamedCommands.getCommand("TakeCoralAuto")));
    m_driverController.leftTrigger(0.5).and(()-> {return checkIntake(13);})
      .whileTrue(NamedCommands.getCommand("CoralIntake").withDeadline(m_swerve.goToIntake(13)).andThen(NamedCommands.getCommand("TakeCoralAuto")));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(1, true);}).whileTrue(m_swerve.goToReef(17, true, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(1, true);}).whileTrue((m_swerve.goToReef(17, true, 3)
              .alongWith(NamedCommands.getCommand("CoralStage3"))));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(1, true);}).whileTrue(m_swerve.goToReef(17, false, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(1, true);}).whileTrue((m_swerve.goToReef(17, false, 3)
              .alongWith(NamedCommands.getCommand("CoralStage3"))));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(2, true);}).whileTrue(m_swerve.goToReef(18, true, 4)
              .alongWith(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(2, true);}).whileTrue(m_swerve.goToReef(18, true, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(2, true);}).whileTrue(m_swerve.goToReef(18, false, 4)
              .alongWith(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(2, true);}).whileTrue(m_swerve.goToReef(18, false, 3)
              .andThen(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(3, true);}).whileTrue(m_swerve.goToReef(19, true, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(3, true);}).whileTrue(m_swerve.goToReef(19, true, 3)
              .alongWith(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(3, true);}).whileTrue(m_swerve.goToReef(19, false, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(3, true);}).whileTrue(m_swerve.goToReef(19, false, 3)
              .alongWith(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(4, true);}).whileTrue(m_swerve.goToReef(20, true, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(4, true);}).whileTrue(m_swerve.goToReef(20, true, 3)
              .alongWith(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(4, true);}).whileTrue(m_swerve.goToReef(20, false, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(4, true);}).whileTrue(m_swerve.goToReef(20, false, 3)
              .alongWith(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(5, true);}).whileTrue(m_swerve.goToReef(21, true, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(5, true);}).whileTrue(m_swerve.goToReef(21, true, 3)
              .alongWith(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(5, true);}).whileTrue(m_swerve.goToReef(21, false, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(5, true);}).whileTrue(m_swerve.goToReef(21, false, 3)
              .alongWith(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(6, true);}).whileTrue(m_swerve.goToReef(22, true, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(6, true);}).whileTrue(m_swerve.goToReef(22, true, 3)
              .alongWith(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(6, true);}).whileTrue(m_swerve.goToReef(22, false, 4)
              .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(6, true);}).whileTrue(m_swerve.goToReef(22, false, 3)
              .alongWith(NamedCommands.getCommand("CoralStage3")));
    
    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(1, true);}).whileTrue(m_swerve.goToAlgae(17, 3)
          .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(1, true);}).whileTrue(m_swerve.goToAlgae(17, 2)
          .andThen(NamedCommands.getCommand("Algae23")));
    
    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(2, true);}).whileTrue(m_swerve.goToAlgae(18, 3)
          .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(2, true);}).whileTrue(m_swerve.goToAlgae(18, 2)
          .andThen(NamedCommands.getCommand("Algae23")));
          
    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(3, true);}).whileTrue(m_swerve.goToAlgae(19, 3)
          .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(3, true);}).whileTrue(m_swerve.goToAlgae(19, 2)
          .andThen(NamedCommands.getCommand("Algae23")));

    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(4, true);}).whileTrue(m_swerve.goToAlgae(20, 3)
          .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(4, true);}).whileTrue(m_swerve.goToAlgae(20, 2)
          .andThen(NamedCommands.getCommand("Algae23")));

    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(5, true);}).whileTrue(m_swerve.goToAlgae(21, 3)
          .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(5, true);}).whileTrue(m_swerve.goToAlgae(21, 2)
          .andThen(NamedCommands.getCommand("Algae23")));

    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(6, true);}).whileTrue(m_swerve.goToAlgae(22, 3)
          .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(6, true);}).whileTrue(m_swerve.goToAlgae(22, 2)
          .andThen(NamedCommands.getCommand("Algae23")));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(1, false);}).whileTrue(m_swerve.goToReef(8, true, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(1, false);}).whileTrue(m_swerve.goToReef(8, true, 3)
      .alongWith(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(1, false);}).whileTrue(m_swerve.goToReef(8, false, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(1, false);}).whileTrue(m_swerve.goToReef(8, false, 3)
      .alongWith(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(2, false);}).whileTrue(m_swerve.goToReef(7, true, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(2, false);}).whileTrue(m_swerve.goToReef(7, true, 3)
      .alongWith(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(2, false);}).whileTrue(m_swerve.goToReef(7, false, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(2, false);}).whileTrue(m_swerve.goToReef(7, false, 3)
      .alongWith(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(3, false);}).whileTrue(m_swerve.goToReef(6, true, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(3, false);}).whileTrue(m_swerve.goToReef(6, true, 3)
      .alongWith(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(3, false);}).whileTrue(m_swerve.goToReef(6, false, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(3, false);}).whileTrue(m_swerve.goToReef(6, false, 3)
      .alongWith(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(4, false);}).whileTrue(m_swerve.goToReef(11, true, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(4, false);}).whileTrue(m_swerve.goToReef(11, true, 3)
      .alongWith(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(4, false);}).whileTrue(m_swerve.goToReef(11, false, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(4, false);}).whileTrue(m_swerve.goToReef(11, false, 3)
      .alongWith(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(5, false);}).whileTrue(m_swerve.goToReef(10, true, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(5, false);}).whileTrue(m_swerve.goToReef(10, true, 3)
      .alongWith(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(5, false);}).whileTrue(m_swerve.goToReef(10, false, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(5, false);}).whileTrue(m_swerve.goToReef(10, false, 3)
      .alongWith(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(6, false);}).whileTrue(m_swerve.goToReef(9, true, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.x().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(6, false);}).whileTrue(m_swerve.goToReef(9, true, 3)
      .alongWith(NamedCommands.getCommand("CoralStage3")));
    m_driverController.a().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(6, false);}).whileTrue(m_swerve.goToReef(9, false, 4)
      .andThen(NamedCommands.getCommand("CoralStage4")));
    m_driverController.b().and(() -> {return !isAlgaeSelected;}).and(() -> {return checkReef(6, false);}).whileTrue(m_swerve.goToReef(9, false, 3)
      .alongWith(NamedCommands.getCommand("CoralStage3")));

    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(1, false);}).whileTrue(m_swerve.goToAlgae(8, 3)
      .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(1, false);}).whileTrue(m_swerve.goToAlgae(8, 2)
      .andThen(NamedCommands.getCommand("Algae23")));

    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(2, false);}).whileTrue(m_swerve.goToAlgae(7, 3)
      .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(2, false);}).whileTrue(m_swerve.goToAlgae(7, 2)
      .andThen(NamedCommands.getCommand("Algae23")));
      
    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(3, false);}).whileTrue(m_swerve.goToAlgae(6, 3)
      .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(3, false);}).whileTrue(m_swerve.goToAlgae(6, 2)
      .andThen(NamedCommands.getCommand("Algae23")));

    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(4, false);}).whileTrue(m_swerve.goToAlgae(11, 3)
      .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(4, false);}).whileTrue(m_swerve.goToAlgae(11, 2)
      .andThen(NamedCommands.getCommand("Algae23")));

    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(5, false);}).whileTrue(m_swerve.goToAlgae(10, 3)
      .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(5, false);}).whileTrue(m_swerve.goToAlgae(10, 2)
      .andThen(NamedCommands.getCommand("Algae23")));

    m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(6, false);}).whileTrue(m_swerve.goToAlgae(9, 3)
      .andThen(NamedCommands.getCommand("Algae34")));
    m_driverController.x().and(() -> {return isAlgaeSelected;}).and(() -> {return checkReef(6, false);}).whileTrue(m_swerve.goToAlgae(9, 2)
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
}
