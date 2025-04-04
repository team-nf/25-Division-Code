package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoConstants;
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

    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
      .getStructTopic("3dSim/fakeRobot", Pose3d.struct).publish();

    private final double initialDriveMultiplier = 0.7;
    private double driveMultiplier = initialDriveMultiplier;

    private final Pose2d blueNetPose2d = new Pose2d(7.6, 6.5, new Rotation2d(Units.degreesToRadians(0)));
    private final Pose2d redNetPose2d = new Pose2d(10, 2, new Rotation2d(180));

    private final SwerveRequest.RobotCentric roboDrive = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.035)
    .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.FieldCentricFacingAngle driveToPoint = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(MaxSpeed * 0.035)
    .withDriveRequestType(DriveRequestType.Velocity);

    PIDController autoControllerX = new PIDController(
        1.0, 0.2, 0.2);
    
    PIDController autoControllerY = new PIDController(
        1.0, 0.2, 0.2);

    private final PhoenixPIDController headingController = new PhoenixPIDController(2.5, 0., 0.);

    private final double errorLimit = 0.02;

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

        if (Robot.isReal())
        {
            updateOdometryWithLL_mt2("limelight-r");
            updateOdometryWithLL_mt2("limelight-l");
            //updateOdometryWithLL_mt1();
            publisher.set(new Pose3d(getState().Pose.getX(),getState().Pose.getY(),0, new Rotation3d(getState().Pose.getRotation())));
        }
        String stat = SmartDashboard.getString("MechState", "Closed");
        if(java.util.Arrays.asList("ThrowAlgaeNet", "CoralStage4", "CoralStage3", "CoralStage2")
               .contains(stat)) driveMultiplier = initialDriveMultiplier/2;
        else if (stat == "Algae23" || stat == "Algae34") driveMultiplier = initialDriveMultiplier/1.1;
        else if(0 <= SmartDashboard.getNumber("Elevator/ElevatorHeight", -10) && 0.6 >= SmartDashboard.getNumber("Elevator/ElevatorHeight", -10)) 
                driveMultiplier = initialDriveMultiplier;
        else driveMultiplier = initialDriveMultiplier/2;
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
            MetersPerSecond.of(4.5).in(MetersPerSecond), MetersPerSecondPerSecond.of(3.5).in(MetersPerSecondPerSecond),
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
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d(3,3,0, new Rotation3d(0,0,0)));
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
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d(3,3,0, new Rotation3d(0,0,0)));
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
        
        return goToPointPID(reefTargetPose2d);
    }

    public Command goToReefAuto(int id, boolean isLeft, int stage)
        {
            Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d(3,3,0, new Rotation3d(0,0,0)));
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

    public Command goToReefWithMarker(int id, boolean isLeft, int stage)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d(3,3,0, new Rotation3d(0,0,0)));
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d aprilTagTargetPose = new Pose2d();
        Pose2d reefTargetPose2d = new Pose2d();

        aprilTagTargetPose = aprilTagPose2d.transformBy(AutoConstants.RobotPosByTag);

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

        return AutoBuilder.pathfindToPose(aprilTagTargetPose, getConstraints()).andThen(AutoBuilder.pathfindToPose(reefTargetPose2d, getConstraints()));
    }

    public Command goToReefWithPath(int id, boolean isLeft, int stage)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d(3,3,0, new Rotation3d(0,0,0)));
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d aprilTagTargetPose = new Pose2d();
        Pose2d reefTargetPose2d = new Pose2d();

        aprilTagTargetPose = aprilTagPose2d.transformBy(AutoConstants.RobotPosByTag);

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

        aprilTagTargetPose = new Pose2d(new Translation2d(14,1.5), new Rotation2d(Units.degreesToRadians(120)));
        reefTargetPose2d =   new Pose2d(new Translation2d(13,2.5), new Rotation2d(Units.degreesToRadians(120)));

        
        SmartDashboard.putString("ATarget", aprilTagPose2d.toString());
        SmartDashboard.putString("RTarget", reefTargetPose2d.toString());
        
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile("Test2");
        } catch (Exception e) {
            return null; // Handle the error appropriately
        }

        return AutoBuilder.pathfindThenFollowPath( 
            path
            , getConstraints());
    }

    public Command goToReefWithPid(int id, boolean isLeft, int stage)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d(3,3,0, new Rotation3d(0,0,0)));
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d aprilTagTargetPose = new Pose2d();
        Pose2d reefTargetPose2d = new Pose2d();

        aprilTagTargetPose = aprilTagPose2d.transformBy(AutoConstants.RobotPosByTag);

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

        aprilTagTargetPose = new Pose2d(new Translation2d(14,1.5), new Rotation2d(Units.degreesToRadians(120)));
        reefTargetPose2d =   new Pose2d(new Translation2d(13,2.5), new Rotation2d(Units.degreesToRadians(120)));

        
        SmartDashboard.putString("ATarget", aprilTagPose2d.toString());
        SmartDashboard.putString("RTarget", reefTargetPose2d.toString());
        
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile("Test2");
        } catch (Exception e) {
            return null; // Handle the error appropriately
        }

        return AutoBuilder.pathfindThenFollowPath( 
            path
            , getConstraints());
    }

    public Command goToAlgae(int id, int stage)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d(3,3,0, new Rotation3d(0,0,0)));
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

    public Command goToIntake(int id)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d(3,3,0, new Rotation3d(0,0,0)));
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d intakeTargetPose2d = new Pose2d();

        intakeTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.IntakeCoralByTag);

        return AutoBuilder.pathfindToPose(intakeTargetPose2d, getConstraints());
    }

    public Command goToIntakeAuto(int id)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d(3,3,0, new Rotation3d(0,0,0)));
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d intakeTargetPose2d = new Pose2d();

        intakeTargetPose2d = aprilTagPose2d.transformBy(AutoConstants.IntakeCoralByTag);

        return AutoBuilder.pathfindToPose(intakeTargetPose2d, getConstraintsForAuto());
    }

    public Command goToProcessor(int id)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d(3,3,0, new Rotation3d(0,0,0)));
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d aprilTagTargetPose = new Pose2d();
        Pose2d algaeTargetPose2d = new Pose2d();

        aprilTagTargetPose = aprilTagPose2d.transformBy(AutoConstants.RobotPosByTag);

        return AutoBuilder.pathfindToPose(aprilTagTargetPose, getConstraints())
                            .andThen(AutoBuilder.pathfindToPose(algaeTargetPose2d, getConstraints()));
    }

    public Command goToNet(int id)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d(3,3,0, new Rotation3d(0,0,0)));
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d aprilTagTargetPose = new Pose2d();
        Pose2d algaeTargetPose2d = new Pose2d();

        aprilTagTargetPose = aprilTagPose2d.transformBy(AutoConstants.RobotPosByTag);

        return AutoBuilder.pathfindToPose(aprilTagTargetPose, getConstraints())
                            .andThen(AutoBuilder.pathfindToPose(algaeTargetPose2d, getConstraints()));
    }


    public Command goToTag(int id)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d(3,3,0, new Rotation3d(0,0,0)));
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d aprilTagTargetPose = new Pose2d();

        aprilTagTargetPose = aprilTagPose2d.transformBy(AutoConstants.RobotPosByTag);

        return AutoBuilder.pathfindToPose(aprilTagTargetPose, getConstraints());
    }

    public Command goToTagAuto(int id)
    {
        Pose3d aprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(id).orElse(new Pose3d(3,3,0, new Rotation3d(0,0,0)));
        Pose2d aprilTagPose2d = new Pose2d(aprilTagPose.getX(), aprilTagPose.getY(), new Rotation2d(aprilTagPose.getRotation().getZ()));
        Pose2d aprilTagTargetPose = new Pose2d();

        aprilTagTargetPose = aprilTagPose2d.transformBy(AutoConstants.RobotPosByTag);

        return AutoBuilder.pathfindToPose(aprilTagTargetPose, getConstraintsForAuto());
    }
    
    public void updateOdometryWithLL_mt1() {
        boolean doRejectUpdate = false;
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-c");
        
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
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
  
        LimelightHelpers.SetRobotOrientation(limelightName, headingDeg, 0, 0, 0, 0, 0);
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
          addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
        }
    }

    public double getDriveMultiplier()
    {
        return driveMultiplier;
    }

    public Command resetHeading()
    {
      return runOnce(() -> seedFieldCentric());     
    }

    public void registerTelemetry() {
        registerTelemetry(logger::telemeterize);
    }

    public Command setPoseBlueAuto()
    {
        return runOnce(() -> resetPose(new Pose2d(7.2, 7.5, new Rotation2d(Units.degreesToRadians(-90)))));
    }

    public Command setPoseRedAuto()
    {
        return runOnce(() -> resetPose(new Pose2d(10.37, 0.56, new Rotation2d(Units.degreesToRadians(90)))));
    }

    public Command setPose(double x, double y, double theta)
    {
        return runOnce(() -> resetPose(new Pose2d(x, y, new Rotation2d(Units.degreesToRadians(theta)))));
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
        autoControllerX.setSetpoint(pose.getX());
        autoControllerY.setSetpoint(pose.getY());

        return run(() -> {
            if(autoControllerX.getSetpoint() != pose.getX()) autoControllerX.setSetpoint(pose.getX());
            if(autoControllerY.getSetpoint() != pose.getY()) autoControllerY.setSetpoint(pose.getY());

            this.setControl(driveToPoint.withVelocityX(-autoControllerX.calculate(getState().Pose.getX()))
                                        .withVelocityY(-autoControllerY.calculate(getState().Pose.getY()))
                                        .withTargetDirection(pose.getRotation().minus(new Rotation2d(Math.PI))));

        }).until(() -> {
            return Math.abs(autoControllerX.getPositionError()) < errorLimit && Math.abs(autoControllerY.getPositionError()) < errorLimit;
        }).finallyDo(() -> {autoControllerX.reset();autoControllerY.reset();});
    }

    public Command getOut()
    {
        return run(() -> setControl(roboDrive.withVelocityX(-0.4))).withTimeout(0.5);
    }
}