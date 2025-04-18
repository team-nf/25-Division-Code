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
import frc.robot.commands.L4PreCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import java.util.Optional;
import java.util.jar.Attributes.Name;

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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final GripperSubsystem m_gripperSubsystem = new GripperSubsystem();
  // private final FunnelSubsystem m_funnelSubsystem = new FunnelSubsystem();
  
  private final MainMechStateMachine m_mainMech = new MainMechStateMachine(m_armSubsystem, m_elevatorSubsystem,
      m_gripperSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS5Controller m_operatorController = new CommandPS5Controller(
      OperatorConstants.kDriverControllerPort);
  
  private final CommandXboxController m_driverController = new CommandXboxController(1);
  

  private final SendableChooser<Integer> m_reefChooser = new SendableChooser<>();
  private final SendableChooser<Boolean> m_isColorBlue = new SendableChooser<>();
  private final SendableChooser<Boolean> m_isAlgaeMode = new SendableChooser<>();
  private final SendableChooser<Boolean> m_isL34       = new SendableChooser<>();
  private final SendableChooser<Boolean> m_isAutoThrow = new SendableChooser<>();
  private final SendableChooser<Integer> m_checkIntake = new SendableChooser<>();
  private final SendableChooser<Integer> m_autoChooser = new SendableChooser<>();


  public final CommandSwerveDrivetrain m_swerve = TunerConstants.createDrivetrain();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(m_swerve.getMaxSpeed() * 0.05).withRotationalDeadband(m_swerve.getMaxAngularRate() * 0.015) // Add a
                                                                                                                // 10%
                                                                                                                // deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(m_swerve.getMaxSpeed() * 0.05).withRotationalDeadband(m_swerve.getMaxAngularRate() * 0.015) // Add a
                                                                                                                // 10%
                                                                                                                // deadband5
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final double kAngle = 0.2;
  private final double kDrive = 0.8;

  private double selectedReef = -1;
  private boolean isBlueSelected = true;
  private boolean isAlgaeSelected = false;
  private boolean isL34Selected = true;
  private double selectedReefID = -1;
  private boolean reefAutoThrow = true;
  private int selectedIntake = 1;

  private boolean algaeChangeFlag = false;

  private SlewRateLimiter d1Filter = new SlewRateLimiter(2.8);
  private SlewRateLimiter d2Filter = new SlewRateLimiter(2.8);
  private SlewRateLimiter rFilter = new SlewRateLimiter(10);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings

    m_swerve.initializeSomeStuff();

    NamedCommands.registerCommand("Closed", new ClosedCmd(m_mainMech));
    NamedCommands.registerCommand("FullyClosed", new ClosedFullyCmd(m_mainMech));
    NamedCommands.registerCommand("CoralIntake", new CoralIntakeCmd(m_mainMech));
    NamedCommands.registerCommand("CoralCarry", new CoralCarryCmd(m_mainMech));
    NamedCommands.registerCommand("CoralStage1", new L1Cmd(m_mainMech));
    NamedCommands.registerCommand("CoralStage2", new L2Cmd(m_mainMech));
    NamedCommands.registerCommand("CoralStage3", new L3Cmd(m_mainMech));
    NamedCommands.registerCommand("CoralStage4", new L4Cmd(m_mainMech));
    NamedCommands.registerCommand("Algae23", new Algae23Cmd(m_mainMech));
    NamedCommands.registerCommand("Algae34", new Algae34Cmd(m_mainMech));
    NamedCommands.registerCommand("AlgaeCarry", new AlgaeCarryCmd(m_mainMech));
    NamedCommands.registerCommand("AlgaeNet", new AlgaeNetCmd(m_mainMech));
    NamedCommands.registerCommand("AlgaeProcessor", new AlgaeProCmd(m_mainMech));
    NamedCommands.registerCommand("AlgaeGround",    new AlgaeGroundCmd(m_mainMech));

    NamedCommands.registerCommand("TakeCoralAuto", m_gripperSubsystem.TakeCoralAutoCommand());
    NamedCommands.registerCommand("ThrowCoralAuto", m_gripperSubsystem.ThrowCoralAutoCommand());

    configureBindings();

    m_swerve.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_swerve.applyRequest(() -> drive
            .withVelocityX(-d1Filter.calculate(m_driverController.getLeftY()) * m_swerve.getMaxSpeed()
                * m_swerve.getDriveMultiplier() * kDrive) // Drive forward with negative Y (forward)
            .withVelocityY(-d2Filter.calculate(m_driverController.getLeftX()) * m_swerve.getMaxSpeed()
                * m_swerve.getDriveMultiplier() * kDrive) // Drive left with negative X (left)
            .withRotationalRate(
                -m_driverController.getRightX() * m_swerve.getMaxAngularRate() * kAngle) // Drive
                                                                                                            // counterclockwise
                                                                                                            // with
                                                                                                            // negative
                                                                                                            // X (left)
        ));

    m_reefChooser.setDefaultOption("1", 1);
    m_reefChooser.addOption("2", 2);
    m_reefChooser.addOption("3", 3);
    m_reefChooser.addOption("4", 4);
    m_reefChooser.addOption("5", 5);
    m_reefChooser.addOption("6", 6);

    m_autoChooser.setDefaultOption("Left", 1);
    m_autoChooser.setDefaultOption("Middle", 2);
    m_autoChooser.setDefaultOption("Right", 3);

    m_isAutoThrow.setDefaultOption("Yes", true);
    m_isAutoThrow.addOption("No", false);

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
    } else {
      m_isColorBlue.setDefaultOption("Blue", true);
      m_isColorBlue.addOption("Red", false);
    }

    m_isAlgaeMode.setDefaultOption("Coral", false);
    m_isAlgaeMode.addOption("Algae", true);

    m_isL34.setDefaultOption("L: 3,4", true);
    m_isL34.addOption("L: 1,2", false);

    m_checkIntake.setDefaultOption("1", 1);
    m_checkIntake.setDefaultOption("2", 2);
    m_checkIntake.setDefaultOption("12", 12);
    m_checkIntake.setDefaultOption("13", 13);

    SmartDashboard.putData("ReefN", m_reefChooser);
    SmartDashboard.putData("AutoThrow??", m_isAutoThrow);
    SmartDashboard.putData("ColorSelect", m_isColorBlue);
    SmartDashboard.putData("AlgaeMode", m_isAlgaeMode);
    SmartDashboard.putData("StageMode", m_isL34);
    SmartDashboard.putData("IntakeSelect", m_checkIntake);
    SmartDashboard.putData("AutoChooser", m_autoChooser);
    SmartDashboard.putBoolean("AlgaeGroundFinished", true);

    m_mainMech.resetMechanisms();

    setSelectorInfos();
    setHandlers();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    m_operatorController.cross().whileTrue(NamedCommands.getCommand("CoralStage1"));
    m_operatorController.circle().whileTrue(NamedCommands.getCommand("CoralStage2"));
    m_operatorController.square().whileTrue(NamedCommands.getCommand("CoralStage3"));
    m_operatorController.triangle().whileTrue(NamedCommands.getCommand("CoralStage4"));
    
    m_operatorController.povLeft().whileTrue(NamedCommands.getCommand("AlgaeNet")); 

    m_operatorController.L1().whileTrue(m_armSubsystem.turnJ1(0.2));
    m_operatorController.R1().whileTrue(m_armSubsystem.turnJ1(-0.2));
    m_operatorController.L2().whileTrue(m_armSubsystem.turnJ2(0.15));
    m_operatorController.R2().whileTrue(m_armSubsystem.turnJ2(-0.15));
    m_operatorController.povUp().whileTrue(m_elevatorSubsystem.eleSpeedControl(0.1));
    m_operatorController.povDown().whileTrue(m_elevatorSubsystem.eleSpeedControl(-0.1));
    //m_operatorController.touchpad().whileTrue(m_mainMech.resetMechanismsCmd());
    m_operatorController.PS().whileTrue(m_mainMech.resetEncodersCmd());

    m_operatorController.L3().whileTrue(NamedCommands.getCommand("AlgaeGround"));
    m_operatorController.R3().whileTrue(NamedCommands.getCommand("CoralIntake"));

    // -------


    m_driverController.rightBumper()
        .whileTrue(m_swerve.applyRequest(() -> driveRobotCentric
            .withVelocityX(
                -m_driverController.getLeftY() * m_swerve.getMaxSpeed() * m_swerve.getDriveMultiplier() * kDrive * 0.45) // Drive
                                                                                                                        // forward
                                                                                                                        // with
                                                                                                                        // negative
                                                                                                                        // Y
                                                                                                                        // (forward)
            .withVelocityY(
                -m_driverController.getLeftX() * m_swerve.getMaxSpeed() * m_swerve.getDriveMultiplier() * kDrive * 0.45) // Drive
                                                                                                                        // left
                                                                                                                        // with
                                                                                                                        // negative
                                                                                                                        // X
                                                                                                                        // (left)
            .withRotationalRate(-m_driverController.getRightX() * m_swerve.getMaxAngularRate() * kAngle) // Drive
                                                                                                         // counterclockwise
                                                                                                         // with
                                                                                                         // negative X
                                                                                                         // (left)
        ));

    m_driverController.pov(0).onTrue(m_gripperSubsystem.takeAlgae());
    m_driverController.pov(90).onTrue(NamedCommands.getCommand("TakeCoralAuto"));
    m_driverController.pov(180).whileTrue(m_gripperSubsystem.throwAlgae().andThen(m_swerve.getOut())); // .andThen(NamedCommands.getCommand("FullyClosed"))
    m_driverController.pov(270).whileTrue(m_gripperSubsystem.throwCoral().andThen(m_swerve.getOut()));

    m_driverController.start().onTrue(m_swerve.resetHeading());
    m_driverController.rightStick().whileTrue(NamedCommands.getCommand("FullyClosed"));

    // m_driverController.button(4).onTrue(m_swerve.setPose(0.5,2.0,0))

    
    m_driverController.leftTrigger().and(() -> {
      return checkReef(1, true);})
        .whileTrue(m_swerve.goToTagSafe(17).alongWith(NamedCommands.getCommand("FullyClosed")));
    m_driverController.leftTrigger().and(() -> {
      return checkReef(2, true);})
        .whileTrue(m_swerve.goToTagSafe(18).alongWith(NamedCommands.getCommand("FullyClosed")));
    m_driverController.leftTrigger().and(() -> {
      return checkReef(3, true);})
        .whileTrue(m_swerve.goToTagSafe(19).alongWith(NamedCommands.getCommand("FullyClosed")));
    m_driverController.leftTrigger().and(() -> {
      return checkReef(4, true);})
        .whileTrue(m_swerve.goToTagSafe(20).alongWith(NamedCommands.getCommand("FullyClosed")));
    m_driverController.leftTrigger().and(() -> {
      return checkReef(5, true);})
        .whileTrue(m_swerve.goToTagSafe(21).alongWith(NamedCommands.getCommand("FullyClosed")));
    m_driverController.leftTrigger().and(() -> {
      return checkReef(6, true);})
        .whileTrue(m_swerve.goToTagSafe(22).alongWith(NamedCommands.getCommand("FullyClosed")));
    m_driverController.leftTrigger().and(() -> {
      return checkReef(1, false);})
        .whileTrue(m_swerve.goToTagSafe(8).alongWith(NamedCommands.getCommand("FullyClosed")));
    m_driverController.leftTrigger().and(() -> {
      return checkReef(2, false);})
        .whileTrue(m_swerve.goToTagSafe(7).alongWith(NamedCommands.getCommand("FullyClosed")));
    m_driverController.leftTrigger().and(() -> {
      return checkReef(3, false);})
        .whileTrue(m_swerve.goToTagSafe(6).alongWith(NamedCommands.getCommand("FullyClosed")));
    m_driverController.leftTrigger().and(() -> {
      return checkReef(4, false);})
        .whileTrue(m_swerve.goToTagSafe(11).alongWith(NamedCommands.getCommand("FullyClosed")));
    m_driverController.leftTrigger().and(() -> {
      return checkReef(5, false);})
        .whileTrue(m_swerve.goToTagSafe(10).alongWith(NamedCommands.getCommand("FullyClosed")));
    m_driverController.leftTrigger().and(() -> {
      return checkReef(6, false);})
        .whileTrue(m_swerve.goToTagSafe(9).alongWith(NamedCommands.getCommand("FullyClosed")));

    // CORAL MODE -->

    m_driverController.back().and(() -> {return !isAlgaeSelected;}).
              whileTrue(NamedCommands.getCommand("CoralIntake"));
    
    m_driverController.leftBumper().and(() -> {return checkIntake(1);})
        .whileTrue(autoIntake(1)
              .andThen(NamedCommands.getCommand("CoralCarry").onlyIf(m_gripperSubsystem::hasCoral)));

    m_driverController.leftBumper().and(() -> {return checkIntake(2);})
        .whileTrue(autoIntake(2)
            .andThen(NamedCommands.getCommand("CoralCarry").onlyIf(m_gripperSubsystem::hasCoral)));
            
    m_driverController.leftBumper().and(() -> {return checkIntake(12);})
        .whileTrue(autoIntake(12)
            .andThen(NamedCommands.getCommand("CoralCarry").onlyIf(m_gripperSubsystem::hasCoral)));    
            
    m_driverController.leftBumper().and(() -> {return checkIntake(13);})
        .whileTrue(autoIntake(13)
            .andThen(NamedCommands.getCommand("CoralCarry").onlyIf(m_gripperSubsystem::hasCoral)));
    
    m_driverController.leftBumper().onTrue(NamedCommands.getCommand("TakeCoralAuto"));

    // 17 - L_34
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(1, true);})
      .whileTrue(autoReefPoseS4L(17));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(1, true);})
      .whileTrue(autoReefPoseS4R(17));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(1, true);})
      .whileTrue(autoReefPoseS3L(17));  
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(1, true);})
      .whileTrue(autoReefPoseS3R(17));
    // 17 - L_12
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(1, true);})
      .whileTrue(autoReefPoseS2L(17));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(1, true);})
      .whileTrue(autoReefPoseS2R(17));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(1, true);})
      .whileTrue(autoReefPoseS1L(17));  
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(1, true);})
      .whileTrue(autoReefPoseS1R(17));

    //---------

    // 18 - L_34
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(2, true);})
      .whileTrue(autoReefPoseS4L(18));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(2, true);})
      .whileTrue(autoReefPoseS4R(18));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(2, true);})
      .whileTrue(autoReefPoseS3L(18));  
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(2, true);})
      .whileTrue(autoReefPoseS3R(18));
    // 18 - L_12
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(2, true);})
      .whileTrue(autoReefPoseS2L(18));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(2, true);})
      .whileTrue(autoReefPoseS2R(18));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(2, true);})
      .whileTrue(autoReefPoseS1L(18));  
    m_driverController.b().and(() -> {
        return (!isAlgaeSelected) && !isL34Selected && checkReef(2, true);})
        .whileTrue(autoReefPoseS1R(18));

        

    //---------

    // 19 - L_34
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(3, true);})
      .whileTrue(autoReefPoseS4L(19));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(3, true);})
      .whileTrue(autoReefPoseS4R(19));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(3, true);})
      .whileTrue(autoReefPoseS3L(19));  
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(3, true);})
      .whileTrue(autoReefPoseS3R(19));
    // 19 - L_12
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(3, true);})
      .whileTrue(autoReefPoseS2L(19));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(3, true);})
      .whileTrue(autoReefPoseS2R(19));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(3, true);})
      .whileTrue(autoReefPoseS1L(19));  
    m_driverController.b().and(() -> {
        return (!isAlgaeSelected) && !isL34Selected && checkReef(3, true);})
        .whileTrue(autoReefPoseS1R(19));

        
    //---------

    // 20 - L_34
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(4, true);})
      .whileTrue(autoReefPoseS4L(20));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(4, true);})
      .whileTrue(autoReefPoseS4R(20));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(4, true);})
      .whileTrue(autoReefPoseS3L(20));  
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(4, true);})
      .whileTrue(autoReefPoseS3R(20));
    // 20 - L_12
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(4, true);})
      .whileTrue(autoReefPoseS2L(20));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(4, true);})
      .whileTrue(autoReefPoseS2R(20));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(4, true);})
      .whileTrue(autoReefPoseS1L(20));  
    m_driverController.b().and(() -> {
        return (!isAlgaeSelected) && !isL34Selected && checkReef(4, true);})
        .whileTrue(autoReefPoseS1R(20));


        
    //---------

    // 21 - L_34
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(5, true);})
      .whileTrue(autoReefPoseS4L(21));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(5, true);})
      .whileTrue(autoReefPoseS4R(21));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(5, true);})
      .whileTrue(autoReefPoseS3L(21));  
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(5, true);})
      .whileTrue(autoReefPoseS3R(21));
    // 21 - L_12
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(5, true);})
      .whileTrue(autoReefPoseS2L(21));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(5, true);})
      .whileTrue(autoReefPoseS2R(21));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(5, true);})
      .whileTrue(autoReefPoseS1L(21));  
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(5, true);})
      .whileTrue(autoReefPoseS1R(21));
    //---------

    // 22 - L_34
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(6, true);})
      .whileTrue(autoReefPoseS4L(22));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(6, true);})
      .whileTrue(autoReefPoseS4R(22));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(6, true);})
      .whileTrue(autoReefPoseS3L(22));  
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(6, true);})
      .whileTrue(autoReefPoseS3R(22));
    // 22 - L_12
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(6, true);})
      .whileTrue(autoReefPoseS2L(22));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(6, true);})
      .whileTrue(autoReefPoseS2R(22));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(6, true);})
      .whileTrue(autoReefPoseS1L(22));  
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(6, true);})
      .whileTrue(autoReefPoseS1R(22));

    //---------

    // 8 - L_34
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(1, false);
    }).whileTrue(autoReefPoseS4L(8));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(1, false);
    }).whileTrue(autoReefPoseS4R(8));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(1, false);
    }).whileTrue(autoReefPoseS3L(8));
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(1, false);
    }).whileTrue(autoReefPoseS3R(8));
    // 8 - L_12
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(1, false);
    }).whileTrue(autoReefPoseS2L(8));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(1, false);
    }).whileTrue(autoReefPoseS2R(8));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(1, false);
    }).whileTrue(autoReefPoseS1L(8));
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(1, false);
    }).whileTrue(autoReefPoseS1R(8));
    //---------

    // 7 - L_34
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(2, false);
    }).whileTrue(autoReefPoseS4L(7));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(2, false);
    }).whileTrue(autoReefPoseS4R(7));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(2, false);
    }).whileTrue(autoReefPoseS3L(7));
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(2, false);
    }).whileTrue(autoReefPoseS3R(7));
    // 7 - L_12
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(2, false);
    }).whileTrue(autoReefPoseS2L(7));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(2, false);
    }).whileTrue(autoReefPoseS2R(7));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(2, false);
    }).whileTrue(autoReefPoseS1L(7));
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(2, false);
    }).whileTrue(autoReefPoseS1R(7));
    //---------

    // 6 - L_34
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(3, false);
    }).whileTrue(autoReefPoseS4L(6));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(3, false);
    }).whileTrue(autoReefPoseS4R(6));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(3, false);
    }).whileTrue(autoReefPoseS3L(6));
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(3, false);
    }).whileTrue(autoReefPoseS3R(6));
    // 6 - L_12
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(3, false);
    }).whileTrue(autoReefPoseS2L(6));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(3, false);
    }).whileTrue(autoReefPoseS2R(6));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(3, false);
    }).whileTrue(autoReefPoseS1L(6));
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(3, false);
    }).whileTrue(autoReefPoseS1R(6));
    //---------

    // 11 - L_34
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(4, false);
    }).whileTrue(autoReefPoseS4L(11));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(4, false);
    }).whileTrue(autoReefPoseS4R(11));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(4, false);
    }).whileTrue(autoReefPoseS3L(11));
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(4, false);
    }).whileTrue(autoReefPoseS3R(11));
    // 11 - L_12
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(4, false);
    }).whileTrue(autoReefPoseS2L(11));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(4, false);
    }).whileTrue(autoReefPoseS2R(11));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(4, false);
    }).whileTrue(autoReefPoseS1L(11));
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(4, false);
    }).whileTrue(autoReefPoseS1R(11));
    //---------

    // 10 - L_34
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(5, false);
    }).whileTrue(autoReefPoseS4L(10));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(5, false);
    }).whileTrue(autoReefPoseS4R(10));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(5, false);
    }).whileTrue(autoReefPoseS3L(10));
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(5, false);
    }).whileTrue(autoReefPoseS3R(10));
    // 10 - L_12
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(5, false);
    }).whileTrue(autoReefPoseS2L(10));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(5, false);
    }).whileTrue(autoReefPoseS2R(10));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(5, false);
    }).whileTrue(autoReefPoseS1L(10));
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(5, false);
    }).whileTrue(autoReefPoseS1R(10));
    //---------

    // 9 - L_34
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(6, false);
    }).whileTrue(autoReefPoseS4L(9));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(6, false);
    }).whileTrue(autoReefPoseS4R(9));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(6, false);
    }).whileTrue(autoReefPoseS3L(9));
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && isL34Selected && checkReef(6, false);
    }).whileTrue(autoReefPoseS3R(9));
    // 9 - L_12
    m_driverController.y().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(6, false);
    }).whileTrue(autoReefPoseS2L(9));
    m_driverController.a().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(6, false);
    }).whileTrue(autoReefPoseS2R(9));
    m_driverController.x().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(6, false);
    }).whileTrue(autoReefPoseS1L(9));
    m_driverController.b().and(() -> {
      return (!isAlgaeSelected) && !isL34Selected && checkReef(6, false);
    }).whileTrue(autoReefPoseS1R(9));
    //---------

    // CORAL MODE <--> ALGAE MODE

    m_driverController.a().and(() -> {return isAlgaeSelected && !m_gripperSubsystem.hasAlgae();})
              .whileTrue(NamedCommands.getCommand("AlgaeGround"));
    m_driverController.a().and(m_mainMech::isAlgaeGround).onTrue(m_gripperSubsystem.takeAlgae());
    m_driverController.a().and(m_mainMech::isAlgaeGround).and(m_gripperSubsystem::hasAlgae)
                          .whileTrue(NamedCommands.getCommand("AlgaeCarry").alongWith(m_swerve.getOut()));

    m_driverController.back().and(() -> {return isAlgaeSelected;}).whileTrue(NamedCommands.getCommand("AlgaeCarry"));

    m_driverController.x().and(() -> {
      return isAlgaeSelected;
    }).whileTrue(NamedCommands.getCommand("AlgaeNet"));
    m_driverController.b().and(() -> {
      return isAlgaeSelected;
    }).whileTrue(NamedCommands.getCommand("AlgaeProcessor"));

    /*
    m_driverController.leftBumper().and(() -> {
      return isAlgaeSelected;
    }).and(() -> {
      return checkTeam(true);
    }).whileTrue(m_swerve.goToBlueNet().alongWith(NamedCommands.getCommand("FullyClosed")));
    m_driverController.leftBumper().and(() -> {
      return isAlgaeSelected;
    }).and(() -> {
      return checkTeam(false);
    }).whileTrue(m_swerve.goToRedNet().alongWith(NamedCommands.getCommand("FullyClosed")));
    */
    m_driverController.y().and(() -> {
      return isAlgaeSelected;
    }).onTrue(m_gripperSubsystem.takeAlgae());

    m_driverController.y().and(() -> {
      return isAlgaeSelected;
    }).and(() -> {
      return checkReef(1, true);
    })
        .whileTrue(autoReefA23(17));
    m_driverController.y().and(() -> {
      return isAlgaeSelected;
    }).and(() -> {
      return checkReef(2, true);
    })
        .whileTrue(autoReefA34(18));
    m_driverController.y().and(() -> {
      return isAlgaeSelected;
    }).and(() -> {
      return checkReef(3, true);
    })
        .whileTrue(autoReefA23(19));
    m_driverController.y().and(() -> {
      return isAlgaeSelected;
    }).and(() -> {
      return checkReef(4, true);
    })
        .whileTrue(autoReefA34(20));
    m_driverController.y().and(() -> {
      return isAlgaeSelected;
    }).and(() -> {
      return checkReef(5, true);
    })
        .whileTrue(autoReefA23(21));
    m_driverController.y().and(() -> {
      return isAlgaeSelected;
    }).and(() -> {
      return checkReef(6, true);
    })
        .whileTrue(autoReefA34(22));

    m_driverController.y().and(() -> {
      return isAlgaeSelected;
    }).and(() -> {
      return checkReef(1, false);
    })
        .whileTrue(autoReefA23(8));
    m_driverController.y().and(() -> {
      return isAlgaeSelected;
    }).and(() -> {
      return checkReef(2, false);
    })
        .whileTrue(autoReefA34(7));
    m_driverController.y().and(() -> {
      return isAlgaeSelected;
    }).and(() -> {
      return checkReef(3, false);
    })
        .whileTrue(autoReefA23(6));
    m_driverController.y().and(() -> {
      return isAlgaeSelected;
    }).and(() -> {
      return checkReef(4, false);
    })
        .whileTrue(autoReefA34(11));
    m_driverController.y().and(() -> {
      return isAlgaeSelected;
    }).and(() -> {
      return checkReef(5, false);
    })
        .whileTrue(autoReefA23(10));
    m_driverController.y().and(() -> {
      return isAlgaeSelected;
    }).and(() -> {
      return checkReef(6, false);
    })
        .whileTrue(autoReefA34(9));

    // ALGAE MODE <--

    /*
     * 
     * m_driverController.y().and(() -> {return isAlgaeSelected;}).and(() -> {return
     * checkReef(1, false);}).whileTrue(m_swerve.goToAlgae(8, 3)
     * .andThen(NamedCommands.getCommand("Algae34")));
     * m_driverController.x().and(() -> {return isAlgaeSelected;}).and(() -> {return
     * checkReef(1, false);}).whileTrue(m_swerve.goToAlgae(8, 2)
     * .andThen(NamedCommands.getCommand("Algae23")));
     * 
     */

    // Climb kontrolleri - basit manuel kontrol
    // m_operatorController.create().whileTrue(m_climbSubsystem.windCommand()); // Yukarı - Create butonu
    // m_operatorController.options().whileTrue(m_climbSubsystem.unwindCommand()); // Aşağı - Options butonu
    
    // // Climb durdurma
    // m_operatorController.touchpad().and(m_operatorController.create()).onTrue(m_climbSubsystem.stopCommand()); // Durdurma
    
    // // Encoder sıfırlama
    // m_operatorController.touchpad().and(m_operatorController.triangle()).onTrue(m_climbSubsystem.resetEncoderCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public boolean checkReef(int reefTag, boolean isBlue) {
    if (isBlue)
      reefTag += 16;
    else {
      reefTag = 9 - reefTag;
      if (reefTag <= 5)
        reefTag += 6;
    }
    return ((selectedReefID == reefTag) && (isBlueSelected == isBlue));
  }

  public boolean checkTeam(boolean isBlue) {
    return isBlueSelected == isBlue;
  }

  public void resetEncoders() {
    m_armSubsystem.resetEncoders();
  }

  public void putSelectedReefID() {
    SmartDashboard.putNumber("SelectedReef", selectedReef);
    SmartDashboard.putNumber("SelectedReefID", selectedReefID);
    SmartDashboard.putNumber("SelectedIntakeID", selectedIntake);
    SmartDashboard.putBoolean("IsBlueSelected", isBlueSelected);
    SmartDashboard.putBoolean("IsAlgaeSelected", isAlgaeSelected);
    SmartDashboard.putBoolean("IsL34Selected", isL34Selected);

  }

  public void setSelectorInfos() {
    isAlgaeSelected = m_isAlgaeMode.getSelected().booleanValue();
    isL34Selected = m_isL34.getSelected().booleanValue();
    isBlueSelected = m_isColorBlue.getSelected().booleanValue();
    reefAutoThrow = m_isAutoThrow.getSelected().booleanValue();
    selectedIntake = m_checkIntake.getSelected().intValue();

    selectedReef = m_reefChooser.getSelected().intValue();
    selectedReefID = selectedReef;
    if (isBlueSelected)
      selectedReefID += 16;
    else {
      selectedReefID = 9 - selectedReefID;
      if (selectedReefID <= 5)
        selectedReefID += 6;
    }
  }

  public void stageModeHandler(boolean a)
  {
    isL34Selected = a;
  }

  public void teamColorHandler(boolean a)
  {
    isBlueSelected = a;

    selectedReefID = selectedReef;
    if (isBlueSelected)
      selectedReefID += 16;
    else {
      selectedReefID = 9 - selectedReefID;
      if (selectedReefID <= 5)
        selectedReefID += 6;
    }
  }

  public void algaeModeHandler(boolean a)
  {
    isAlgaeSelected = a;
  }

  public void autoThrowHandler(boolean a)
  {
    reefAutoThrow = a;
  }
  
  public void intakeHandler(int n) {
    selectedIntake = n;
  }

  public void selectReefHandler(double n)
  {
    selectedReef = n;
    selectedReefID = selectedReef;
    if (isBlueSelected)
      selectedReefID += 16;
    else {
      selectedReefID = 9 - selectedReefID;
      if (selectedReefID <= 5)
        selectedReefID += 6;
    }
  }

  
  public void setHandlers() 
  {
    m_isAlgaeMode.onChange(this::algaeModeHandler);
    m_isL34.onChange(this::stageModeHandler);
    m_isColorBlue.onChange(this::teamColorHandler);
    m_isAutoThrow.onChange(this::autoThrowHandler);
    m_reefChooser.onChange(this::selectReefHandler);
    m_checkIntake.onChange(this::intakeHandler);
  }

  public boolean checkIntake(int n) {
    return n == selectedIntake;
  }

  public void containerPeriodic() {
    putSelectedReefID();
    changeAlgaeModeFunction();
    algaeModeLLControl();
    m_mainMech.periodic();
  }

  public Command autoReefPoseS4L(int id) {
    return m_swerve.goToTagL(id).andThen(NamedCommands.getCommand("CoralStage4")
        .withDeadline(m_swerve.goToReefWithPID(id, true, 4)))
          .andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(this::isAutoThrow));
  }

  public Command autoReefPoseS4R(int id) {
    return m_swerve.goToTagR(id).andThen(NamedCommands.getCommand("CoralStage4")
        .withDeadline(m_swerve.goToReefWithPID(id, false, 4)))
        .andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(this::isAutoThrow));

  }

  public Command autoReefPoseS3L(int id) {
    return m_swerve.goToTagL(id).andThen(NamedCommands.getCommand("CoralStage3")
        .withDeadline(m_swerve.goToReefWithPID(id, true, 3)))
        .andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(this::isAutoThrow));

  }

  public Command autoReefPoseS3R(int id) {
    return m_swerve.goToTagR(id).andThen(NamedCommands.getCommand("CoralStage3")
        .withDeadline(m_swerve.goToReefWithPID(id, false, 3)))
        .andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(this::isAutoThrow));

  }

  public Command autoReefPoseS2L(int id) {
    return m_swerve.goToTagL(id).andThen(NamedCommands.getCommand("CoralStage2")
        .withDeadline(m_swerve.goToReefWithPID(id, true, 3)))
        .andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(this::isAutoThrow));

  }

  public Command autoReefPoseS2R(int id) {
    return m_swerve.goToTagR(id).andThen(NamedCommands.getCommand("CoralStage2")
        .withDeadline(m_swerve.goToReefWithPID(id, false, 3)))
        .andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(this::isAutoThrow));

  }

  public Command autoReefPoseS1L(int id) {
    return m_swerve.goToTag(id).andThen(NamedCommands.getCommand("CoralStage1")
        .withDeadline(m_swerve.goToReefWithPID(id, true, 1)))
        .andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(this::isAutoThrow));

  }

  public Command autoReefPoseS1R(int id) {
    return m_swerve.goToTag(id).andThen(NamedCommands.getCommand("CoralStage1")
        .withDeadline(m_swerve.goToReefWithPID(id, false, 1)))
        .andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(this::isAutoThrow));

  }

  public Command autoReefA23(int id) {
    return NamedCommands.getCommand("Algae23").withDeadline(m_swerve.goToAlgae(id, 2))
        .andThen(m_swerve.goToAlgaeWithPID(id).until(m_gripperSubsystem::hasAlgae)).andThen(m_swerve.getOut())
        .andThen(NamedCommands.getCommand("AlgaeCarry"));
  }

  public Command autoReefA34(int id) {
    return NamedCommands.getCommand("Algae34").withDeadline(m_swerve.goToAlgae(id, 3))
        .andThen(m_swerve.goToAlgaeWithPID(id).until(m_gripperSubsystem::hasAlgae)).andThen(m_swerve.getOut())
        .andThen(NamedCommands.getCommand("AlgaeCarry"));
  }

  public Command autoIntake(int id)
  {
    return (NamedCommands.getCommand("CoralIntake").withDeadline(m_swerve.goToIntake(id)))
                .andThen(m_swerve.goToIntakeWithPid(id));
  }

  public Command autoIntakeForAuto(int id)
  {
    return (NamedCommands.getCommand("CoralIntake").withDeadline(m_swerve.goToIntake(id))
                .andThen(m_swerve.goToIntakeWithPid(id).alongWith(NamedCommands.getCommand("TakeCoralAuto"))));
  }

  public Command getAutonomousCommandBlue_1() {
    // An example command will be run in autonomous
    return m_swerve.setPoseBlueAuto()
        .andThen(autoReefPoseS4R(20).withTimeout(5).andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(m_gripperSubsystem::hasCoral)))
        .andThen(autoIntakeForAuto(13))
        .andThen(autoReefPoseS4L(19).withTimeout(5).andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(m_gripperSubsystem::hasCoral)))
        .andThen(autoIntakeForAuto(13))
        .andThen(autoReefPoseS4R(19).withTimeout(5).andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(m_gripperSubsystem::hasCoral)))
        .andThen(autoIntake(13));
  }

  public Command getAutonomousCommandBlue_2() {
    // An example command will be run in autonomous
    return m_swerve.setPoseBlueAuto_2()
        .andThen(autoReefPoseS4R(21).withTimeout(5).andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(m_gripperSubsystem::hasCoral)))
        .andThen(m_swerve.goToTagAuto(21).andThen(NamedCommands.getCommand("FullyClosed")));
  
  }

  public Command getAutonomousCommandBlue_3() {
    // An example command will be run in autonomous
    return m_swerve.setPoseBlueAuto_3()
        .andThen(autoReefPoseS4L(22).withTimeout(5).andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(m_gripperSubsystem::hasCoral)))
        .andThen(autoIntakeForAuto(12))
        .andThen(autoReefPoseS4R(17).withTimeout(5).andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(m_gripperSubsystem::hasCoral)))
        .andThen(autoIntakeForAuto(12))
        .andThen(autoReefPoseS4L(17).withTimeout(5).andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(m_gripperSubsystem::hasCoral)))
        .andThen(autoIntake(12));
  }

public Command getAutonomousCommandRed_1() {
  // An example command will be run in autonomous
  return m_swerve.setPoseRedAuto()
      .andThen(autoReefPoseS4R(11).withTimeout(5).andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(m_gripperSubsystem::hasCoral)))
      .andThen(autoIntakeForAuto(1))
      .andThen(autoReefPoseS4L(6).withTimeout(5).andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(m_gripperSubsystem::hasCoral)))
      .andThen(autoIntakeForAuto(1))
      .andThen(autoReefPoseS4R(6).withTimeout(5).andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(m_gripperSubsystem::hasCoral)))
      .andThen(autoIntake(1));
}

public Command getAutonomousCommandRed_2() {
  // An example command will be run in autonomous
  return m_swerve.setPoseRedAuto_2()
      .andThen(autoReefPoseS4R(10).withTimeout(5).andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(m_gripperSubsystem::hasCoral)))
      .andThen(m_swerve.goToTagAuto(10).andThen(NamedCommands.getCommand("FullyClosed")));

}

public Command getAutonomousCommandRed_3() {
  // An example command will be run in autonomous
  return m_swerve.setPoseRedAuto_3()
      .andThen(autoReefPoseS4R(9).withTimeout(5).andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(m_gripperSubsystem::hasCoral)))
      .andThen(autoIntakeForAuto(2))
      .andThen(autoReefPoseS4L(8).withTimeout(5).andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(m_gripperSubsystem::hasCoral)))
      .andThen(autoIntakeForAuto(2))
      .andThen(autoReefPoseS4R(8).withTimeout(5).andThen(NamedCommands.getCommand("ThrowCoralAuto").onlyIf(m_gripperSubsystem::hasCoral)))
      .andThen(autoIntake(2));
}

public Command getAutonomousCommand() {
  // An example command will be run in autonomous
  Optional<Alliance> ally = DriverStation.getAlliance();
  if (ally.isPresent()) {
    if (ally.get() == Alliance.Blue) {
      if(m_autoChooser.getSelected().intValue() == 1) return getAutonomousCommandBlue_1();
      else if(m_autoChooser.getSelected().intValue() == 3) return getAutonomousCommandBlue_3();
      else return getAutonomousCommandBlue_2();

    } else
    if(m_autoChooser.getSelected().intValue() == 1) return getAutonomousCommandRed_1();
    else if(m_autoChooser.getSelected().intValue() == 3) return getAutonomousCommandRed_3();
    else return getAutonomousCommandRed_2();  }
  return null;
}

public Command getAutoTest() {
  return autoReefPoseS4R(11)
         .andThen(autoIntakeForAuto(1))
         .andThen(autoReefPoseS4L(6))
         .andThen(autoIntakeForAuto(1))
         .andThen(autoReefPoseS4R(6))
         .andThen(autoIntake(1));
}

public void changeToAlgaeMode()
{
  isAlgaeSelected = true;
}

public void changeToCoralMode()
{
  isAlgaeSelected = false;
}

public boolean isAutoThrow()
{
  return reefAutoThrow || DriverStation.isAutonomous();
}

public void changeAlgaeModeFunction()
{
  if(m_driverController.rightTrigger(0.5).getAsBoolean() && !algaeChangeFlag) 
  {
    algaeChangeFlag = true;
    isAlgaeSelected = !isAlgaeSelected;
  }
  if(!m_driverController.rightTrigger(0.5).getAsBoolean() && algaeChangeFlag) algaeChangeFlag = false;
}

public void algaeModeLLControl()
{
  if(isAlgaeSelected)
  {
    m_swerve.setLimelightLightOn("limelight-r"); 
    m_swerve.setLimelightLightOn("limelight-l"); 
  }
  else
  {
    m_swerve.setLimelightLightOff("limelight-r"); 
    m_swerve.setLimelightLightOff("limelight-l"); 
  }
}

public Command poseTestCommand()
{
  return m_swerve.setPoseBlueAuto_3();
}

}
