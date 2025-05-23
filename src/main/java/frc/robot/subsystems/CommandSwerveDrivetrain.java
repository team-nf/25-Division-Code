package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.Supplier;

import org.w3c.dom.views.DocumentView;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private double MaxSpeed = MetersPerSecond.of(5).in(MetersPerSecond); //TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(3).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
    //  .getStructTopic("3dSim/fakeRobot", Pose3d.struct).publish();

    private final Field2d m_field = new Field2d();

    private final double initialDriveMultiplier = 0.8;
    private double driveMultiplier = initialDriveMultiplier;

    private final Pose2d blueNetPose2d = new Pose2d(7.85, 6.5, new Rotation2d(Units.degreesToRadians(0)));
    private final Pose2d redNetPose2d = new Pose2d(9.9, 2, new Rotation2d(Units.degreesToRadians(180)));

    private final SwerveRequest.RobotCentric roboDrive = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.035)
    .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.FieldCentricFacingAngle driveToPoint = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(MaxSpeed * 0.035)
    .withDriveRequestType(DriveRequestType.Velocity);
    

    PIDController reefPIDX = new PIDController(
        2.0, 1, 0.2);
    
    PIDController reefPIDY = new PIDController(
        2.0, 1, 0.2);

    private final PhoenixPIDController headingController = new PhoenixPIDController(1, 0., 0.);

    PIDController autoControllerX = new PIDController(
        2.7, 1, 0.2);
    
    PIDController autoControllerY = new PIDController(
        2.7, 1, 0.2);

    private final double errorLimit = 0.02;
    private final double errorLimitAuto = 0.06;

    private final SendableChooser<Boolean> m_isLeftVisionEnabled = new SendableChooser<>();
    private final SendableChooser<Boolean> m_isRightVisionEnabled = new SendableChooser<>();

    private boolean isLeftVisionEnabled = true;
    private boolean isRightVisionEnabled = true;

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        if (Robot.isReal() && RobotState.isAutonomous())
        {
            if(isRightVisionEnabled)
            {            
                updateOdometryWithLL("limelight-r");
            }
            if(isLeftVisionEnabled)
            {            
                updateOdometryWithLL("limelight-l");

            }
            //updateOdometryWithLL_mt1();
        }

        if (Robot.isReal() && RobotState.isTeleop() )
        {
            if(isRightVisionEnabled)
            {            
                updateOdometryWithLL("limelight-r");
            }
            if(isLeftVisionEnabled)
            {            
                updateOdometryWithLL("limelight-l");

            }
            //updateOdometryWithLL_mt1();
        }


        SmartDashboard.putBoolean("isAllyBlue", DriverStation.getAlliance().get() == Alliance.Blue);


        String stat = SmartDashboard.getString("MechState", "Closed");
        if(java.util.Arrays.asList("ThrowAlgaeNet", "CoralStage4", "CoralStage3", "CoralStage2")
               .contains(stat)) driveMultiplier = initialDriveMultiplier/2;
        else if (stat == "Algae23" || stat == "Algae34") driveMultiplier = initialDriveMultiplier/1.1;
        else if(0 <= SmartDashboard.getNumber("Elevator/ElevatorHeight", -10) && 0.6 >= SmartDashboard.getNumber("Elevator/ElevatorHeight", -10)) 
                driveMultiplier = initialDriveMultiplier;
        else driveMultiplier = initialDriveMultiplier/2;

        //SmartDashboard.putNumber("PigeonVal", getPigeon2().getRotation2d().getDegrees());
        m_field.setRobotPose(getState().Pose);
        SmartDashboard.putData("Field", m_field);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(4.0, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(4.0, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }    
    }

    public double getMaxSpeed()
    {
        return MaxSpeed;
    }

    public double getMaxAngularRate()
    {
        return MaxAngularRate;
    }

    public PathConstraints getConstraints() {
        return new PathConstraints(
            MetersPerSecond.of(PathConstants.velocity).in(MetersPerSecond), MetersPerSecondPerSecond.of(PathConstants.acceleration).in(MetersPerSecondPerSecond),
            RotationsPerSecond.of(180).in(RadiansPerSecond), RotationsPerSecondPerSecond.of(120).in(RadiansPerSecondPerSecond));
    }

    public PathConstraints getSafeConstraints() {
        return new PathConstraints(
            MetersPerSecond.of(PathConstants.velocity/2).in(MetersPerSecond), MetersPerSecondPerSecond.of(PathConstants.acceleration/2.5).in(MetersPerSecondPerSecond),
            RotationsPerSecond.of(180).in(RadiansPerSecond), RotationsPerSecondPerSecond.of(120).in(RadiansPerSecondPerSecond));
    }

    public PathConstraints getConstraintsForAuto() {
        return new PathConstraints(
            MetersPerSecond.of(3.8).in(MetersPerSecond), MetersPerSecondPerSecond.of(2.3).in(MetersPerSecondPerSecond),
            RotationsPerSecond.of(180).in(RadiansPerSecond), RotationsPerSecondPerSecond.of(120).in(RadiansPerSecondPerSecond));
    }

    public PathConstraints getTestConstraints() {
        return new PathConstraints(
            MetersPerSecond.of(5.0).in(MetersPerSecond), MetersPerSecondPerSecond.of(4.5).in(MetersPerSecondPerSecond),
            RotationsPerSecond.of(180).in(RadiansPerSecond), RotationsPerSecondPerSecond.of(120).in(RadiansPerSecondPerSecond));
    }

    public Command goToReef(int id, boolean isLeft, int stage)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d());
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d reefTargetPose2d = new Pose2d();

        if(stage == 4)
        {
            if(isLeft)
            {
               reefTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.ReefPosS4LByTag);
            }
            else
            {
               reefTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.ReefPosS4RByTag);
            }
        }
        else if(stage == 3)
        {
            if(isLeft)
            {
                reefTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.ReefPosS3LByTag);
            }
            else
            {
                reefTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.ReefPosS3RByTag);
            }
        }
        
        return AutoBuilder.pathfindToPose(reefTargetPose2d, getConstraints());
    }

    public Command goToReefWithPID(int id, boolean isLeft, int stage)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d());
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d reefTargetPose2d = new Pose2d();

        if(stage == 4)
        {
            if(isLeft)
            {
               reefTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.ReefPosS4LByTag);
            }
            else
            {
               reefTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.ReefPosS4RByTag);
            }
        }
        else if(stage == 3)
        {
            if(isLeft)
            {
                reefTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.ReefPosS3LByTag);
            }
            else
            {
                reefTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.ReefPosS3RByTag);
            }
        }
        else if(stage == 1)
        {
            if(isLeft)
            {
                reefTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.ReefPosS1LByTag);
            }
            else
            {
                reefTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.ReefPosS1RByTag);
            }
        }
        
        return goToPointPID(reefTargetPose2d);
    }

    public Command goToReefAuto(int id, boolean isLeft, int stage)
        {
            Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d());
            Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
            Pose2d reefTargetPose2d = new Pose2d();
    
            if(stage == 4)
            {
                if(isLeft)
                {
                   reefTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.ReefPosS4LByTag);
                }
                else
                {
                   reefTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.ReefPosS4RByTag);
                }
            }
            else if(stage == 3)
            {
                if(isLeft)
                {
                    reefTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.ReefPosS3LByTag);
                }
                else
                {
                    reefTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.ReefPosS3RByTag);
                }
            }
    

        //return AutoBuilder.pathfindToPose(aprilTagTargetPose, getConstraints()).andThen(AutoBuilder.pathfindToPose(reefTargetPose2d, getConstraints()));
        return AutoBuilder.pathfindToPose(reefTargetPose2d, getConstraintsForAuto());
    }

    public Command goToAlgae(int id, int stage)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d());
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d algaeTargetPose2d = new Pose2d();

        if(stage == 3)
        {
            algaeTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.Algae3ByTag);
        }
        else if(stage == 2)
        {
            algaeTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.Algae2ByTag);
        }

        return AutoBuilder.pathfindToPose(algaeTargetPose2d, getConstraints());
    }

    public Command goToAlgaeWithPID(int id)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d());
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d algaeTargetPose2d = new Pose2d();

        algaeTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.TakeAlgaeByTag);


        return goToPointPID(algaeTargetPose2d);
    }

    public Command goToIntake(int id)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d());
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d intakeTargetPose2d = new Pose2d();

        intakeTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.IntakeCoralByTag);

        return AutoBuilder.pathfindToPose(intakeTargetPose2d, getConstraints());
    }

    public Command goToIntakeWithPid(int id)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d());
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d intakeTargetPose2d = new Pose2d();

        intakeTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.IntakeCoralByTag);

        return goToPointPIDAuto(intakeTargetPose2d);
    }

    public Command goToIntakeAuto(int id)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d());
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d intakeTargetPose2d = new Pose2d();

        intakeTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.IntakeCoralByTag);

        return AutoBuilder.pathfindToPose(intakeTargetPose2d, getConstraintsForAuto());
    }

    public Command goToTagSafe(int id)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d());
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d aprilTagTargetPose = new Pose2d();

        aprilTagTargetPose = aprilTagPose2d.transformBy(AutoConstants.RobotPosByTagSafe);

        return AutoBuilder.pathfindToPose(aprilTagTargetPose, getSafeConstraints());
    }

    public Command goToTagSafeAuto(int id)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d());
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d aprilTagTargetPose = new Pose2d();

        aprilTagTargetPose = aprilTagPose2d.transformBy(AutoConstants.RobotPosByTagSafe);

        return AutoBuilder.pathfindToPose(aprilTagTargetPose, getSafeConstraints());
    }


    public Command goToTag(int id)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d());
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d aprilTagTargetPose = new Pose2d();

        aprilTagTargetPose = aprilTagPose2d.transformBy(AutoConstants.RobotPosByTag);

        return AutoBuilder.pathfindToPose(aprilTagTargetPose, getConstraints());
    }

    public Command goToTagAuto(int id)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d());
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d aprilTagTargetPose = new Pose2d();

        aprilTagTargetPose = aprilTagPose2d.transformBy(AutoConstants.RobotPosByTag);

        return AutoBuilder.pathfindToPose(aprilTagTargetPose, getSafeConstraints());
    }

    public Command goToTagL(int id)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d());
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d aprilTagTargetPose = new Pose2d();

        aprilTagTargetPose = aprilTagPose2d.transformBy(AutoConstants.RobotPosByTagL);

        return AutoBuilder.pathfindToPose(aprilTagTargetPose, getConstraints());
    }

    public Command goToTagR(int id)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d());
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d aprilTagTargetPose = new Pose2d();

        aprilTagTargetPose = aprilTagPose2d.transformBy(AutoConstants.RobotPosByTagR);

        return AutoBuilder.pathfindToPose(aprilTagTargetPose, getConstraints());
    }

    public void updateOdometryWithLL(String limelightName)
    {
        var driveState = getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
        double driveSpeed = Math.hypot(driveState.Speeds.vxMetersPerSecond, driveState.Speeds.vyMetersPerSecond); 
  
        LimelightHelpers.SetRobotOrientation(limelightName, headingDeg, 0, 0, 0, 0, 0);
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        String mt_state = "null";

        if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2 
                && llMeasurement.avgTagDist < 3.5 && driveSpeed < 2.5) 
        {
          if(llMeasurement.avgTagDist < AutoConstants.MT1_DIST && driveSpeed < 0.8
                    && llMeasurement.rawFiducials[0].ta > AutoConstants.LL_Accuracy_mt1)
          {
          llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
          mt_state = "mt1";
          }
          else mt_state = "mt2"; 
          addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
        }

        SmartDashboard.putString(limelightName + "-LL-State", mt_state);
    }
    
    public void updateOdometryWithLL_mt1(String limelightName) {
        boolean doRejectUpdate = false;
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        
        if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
        {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
            doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
            doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].ta < AutoConstants.LL_Accuracy_mt1) doRejectUpdate = true;

        }

        if(mt1.tagCount == 0)
        {
        doRejectUpdate = true;
        }

        if(!doRejectUpdate)
        {
        setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
        }
    }

    public void updateOdometryWithLL_mt2(String limelightName) {
        var driveState = getState();
        Rotation2d rot =  getState().Pose.getRotation(); 
        double headingDeg = rot.getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
  
        LimelightHelpers.SetRobotOrientation(limelightName, headingDeg, 0, 0, 0, 0, 0);
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.5 
                && llMeasurement.avgTagDist < 2) {

          addVisionMeasurement(new Pose2d(llMeasurement.pose.getTranslation(), rot), llMeasurement.timestampSeconds);
        }
        
    }

    public double getDriveMultiplier()
    {
        return driveMultiplier;
    }

    public Command resetHeading()
    {
      //return runOnce(() -> resetRotation(new Rotation2d(DriverStation.getAlliance().get() == Alliance.Blue ? 0 : Math.PI)));   
      return runOnce(() -> seedFieldCentric());
    }

    public void registerTelemetry() {
        registerTelemetry(logger::telemeterize);
    }
    
    public void resetThePose(Pose2d p)
    {
        resetPose(p);
        addVisionMeasurement(p, kNumConfigAttempts);
    }

    public Command setPoseBlueAuto()
    {
        return runOnce(() -> resetThePose(new Pose2d(7.15, 7.55, new Rotation2d(Units.degreesToRadians(-90))))).andThen(new WaitCommand(0.03));
    }

    public Command setPoseBlueAuto_2()
    {
        return runOnce(() -> resetThePose(new Pose2d(7.15, 4, new Rotation2d(Units.degreesToRadians(180))))).andThen(new WaitCommand(0.03));
    }

    public Command setPoseBlueAuto_3()
    {
        return runOnce(() -> resetThePose(new Pose2d(7.15, 0.45, new Rotation2d(Units.degreesToRadians(90))))).andThen(new WaitCommand(0.03));
    }

    public Command setPoseRedAuto()
    {
        return runOnce(() -> resetThePose(new Pose2d(10.385, 0.45, new Rotation2d(Units.degreesToRadians(90))))).andThen(new WaitCommand(0.03));
    }

    public Command setPoseRedAuto_2()
    {
        return runOnce(() -> resetThePose(new Pose2d(10.385, 4, new Rotation2d(Units.degreesToRadians(0))))).andThen(new WaitCommand(0.03));
    }

    public Command setPoseRedAuto_3()
    {
        return runOnce(() -> resetThePose(new Pose2d(10.385, 7.55, new Rotation2d(Units.degreesToRadians(-90))))).andThen(new WaitCommand(0.03));
    }

    public Command setPose(double x, double y, double theta)
    {
        return runOnce(() -> resetThePose(new Pose2d(x, y, new Rotation2d(Units.degreesToRadians(theta)))));
    }

    public Command goToBlueNet()
    {
        return AutoBuilder.pathfindToPose(blueNetPose2d, getConstraints());
    }


    public Command goToRedNet()
    {
        return AutoBuilder.pathfindToPose(redNetPose2d, getConstraints());
    }

    public Command goToPoint(double x, double y, double angle)
    {
        return AutoBuilder.pathfindToPose(new Pose2d(x,y, new Rotation2d(Units.degreesToRadians(angle))), getConstraints());
    }

    public Command goToPointPID(Pose2d pose)
    {
        driveToPoint.HeadingController = headingController;
        driveToPoint.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        reefPIDX.setSetpoint(pose.getX());
        reefPIDY.setSetpoint(pose.getY());

        return run(() -> {
            if(reefPIDX.getSetpoint() != pose.getX()) reefPIDX.setSetpoint(pose.getX());
            if(reefPIDY.getSetpoint() != pose.getY()) reefPIDY.setSetpoint(pose.getY());

            SmartDashboard.putNumber("PID-XError", reefPIDX.getPositionError());
            SmartDashboard.putNumber("PID-YError", reefPIDY.getPositionError());

            if(Alliance.Blue == DriverStation.getAlliance().get())
            {
                this.setControl(driveToPoint.withVelocityX(reefPIDX.calculate(getState().Pose.getX()))
                .withVelocityY(reefPIDY.calculate(getState().Pose.getY()))
                .withTargetDirection(pose.getRotation()));
            }
            else
            {
                this.setControl(driveToPoint.withVelocityX(-reefPIDX.calculate(getState().Pose.getX()))
                .withVelocityY(-reefPIDY.calculate(getState().Pose.getY()))
                .withTargetDirection(pose.getRotation().minus(new Rotation2d(Math.PI))));
            }

        }).until(() -> {
            return Math.abs(reefPIDX.getPositionError()) < errorLimit 
                    && Math.abs(reefPIDY.getPositionError()) < errorLimit 
                        && driveToPoint.HeadingController.getPositionError() < 3;
        }).finallyDo(() -> {reefPIDX.reset();reefPIDY.reset();});
    }

    public Command goToPointPIDAuto(Pose2d pose)
    {
        driveToPoint.HeadingController = headingController;
        driveToPoint.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        autoControllerX.setSetpoint(pose.getX());
        autoControllerY.setSetpoint(pose.getY());
 
        return run(() -> {
            if(autoControllerX.getSetpoint() != pose.getX()) autoControllerX.setSetpoint(pose.getX());
            if(autoControllerY.getSetpoint() != pose.getY()) autoControllerY.setSetpoint(pose.getY());

            SmartDashboard.putNumber("PID-XError", autoControllerX.getPositionError());
            SmartDashboard.putNumber("PID-YError", autoControllerX.getPositionError());
            if(Alliance.Blue == DriverStation.getAlliance().get())
            {
                this.setControl(driveToPoint.withVelocityX(autoControllerX.calculate(getState().Pose.getX()))
                .withVelocityY(autoControllerY.calculate(getState().Pose.getY()))
                .withTargetDirection(pose.getRotation()));
            }
            else
            {
                this.setControl(driveToPoint.withVelocityX(-autoControllerX.calculate(getState().Pose.getX()))
                .withVelocityY(-autoControllerY.calculate(getState().Pose.getY()))
                .withTargetDirection(pose.getRotation().minus(new Rotation2d(Math.PI))));
            }
            

        }).until(() -> {
            return Math.abs(autoControllerX.getPositionError()) < errorLimitAuto
                    && Math.abs(autoControllerY.getPositionError()) < errorLimitAuto;
        }).finallyDo(() -> {autoControllerX.reset();autoControllerY.reset();stopChassis();});
    }

    public Command getOut()
    {
        return run(() -> setControl(roboDrive.withVelocityX(-0.8))).withTimeout(0.4);
    }

    public void initializeSomeStuff()
    {  
        m_isLeftVisionEnabled.setDefaultOption("Yes", true);
        m_isLeftVisionEnabled.addOption("No", false);
        m_isLeftVisionEnabled.onChange(this::setVisionLeft);
        m_isRightVisionEnabled.setDefaultOption("Yes", true);
        m_isRightVisionEnabled.addOption("No", false);
        m_isRightVisionEnabled.onChange(this::setVisionRight);


        //SmartDashboard.putData("LeftVisionEnable", m_isLeftVisionEnabled);
        //SmartDashboard.putData("RightVisionEnable", m_isRightVisionEnabled);

    }

    public void setVisionLeft(boolean a)
    {
        isLeftVisionEnabled = a;
    }

    public void setVisionRight(boolean a)
    {
        isRightVisionEnabled = a;
    }

    public void stopChassis()
    {
        this.setControl(roboDrive.withVelocityX(0).withVelocityY(0));
    }

    public void setDriveMultiplier(double multiplier)
    {
        driveMultiplier = multiplier;
    }

    public void setLimelightLightOn(String limelightName)
    {
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
    }

    
    public void setLimelightLightOff(String limelightName)
    {
        LimelightHelpers.setLEDMode_ForceOff(limelightName);
    }
}