package com.team841.calliope.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team841.calliope.constants.Field;
import com.team841.calliope.constants.RC;
import com.team841.calliope.constants.Swerve;
import com.team841.calliope.vision.LimelightHelpers;
import com.team841.lib.util.LoggedTunableNumber;
import com.team841.lib.util.atan2.LUT.Atan2LUT;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class Drivetrain extends SwerveDrivetrain implements Subsystem {


    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable networkTablePoses = inst.getTable("Drivetrain Poses");

    StructTopic<Pose2d> limelightTopic = networkTablePoses.getStructTopic("Limelight Front Pose", Pose2d.struct);
    StructTopic<Pose2d> ctreTopic = networkTablePoses.getStructTopic("CTRE Pose", Pose2d.struct);

    StructPublisher<Pose2d> limelightPublisher = limelightTopic.publish();
    StructPublisher<Pose2d> ctrePublisher = ctreTopic.publish();

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest =
            new SwerveRequest.ApplyChassisSpeeds();

    //public ComputeThread compute;

    public Drivetrain(
            SwerveDrivetrainConstants driveTrainConstants,
            double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        this.setOperatorPerspectiveForward(
                RC.isRedAlliance.get() ? new Rotation2d(Math.PI) : new Rotation2d(0));

    /*
    this.compute = new ComputeThread();
    this.compute.start();

     */

        ConfigureMotors();
        configurePathplanner();
    }

    public Drivetrain(
            SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        this.setOperatorPerspectiveForward(
                RC.isRedAlliance.get() ? new Rotation2d(Math.PI) : new Rotation2d(0));

    /*
    this.compute = new ComputeThread();
    this.compute.start();

     */

        ConfigureMotors();
        configurePathplanner();
    }

    private void ConfigureMotors() {
        for (var CurrentModule : this.Modules) {
            CurrentModule.getDriveMotor()
                    .getConfigurator()
                    .apply(
                            new CurrentLimitsConfigs()
                                    .withSupplyCurrentLimit(50)
                                    .withSupplyCurrentLimitEnable(true));
            CurrentModule.getSteerMotor()
                    .getConfigurator()
                    .apply(
                            new CurrentLimitsConfigs()
                                    .withSupplyCurrentLimit(60)
                                    .withSupplyCurrentLimitEnable(true));
        }
    }

    public void emtpyReturn(Pose2d input){
        return;
    }

    public void configurePathplanner() {

        double driveBaseRadius = 0;

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) ->
                        this.setControl(
                                autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
                new HolonomicPathFollowerConfig(
                        new PIDConstants(10, 0, 0),
                        new PIDConstants(8, 0, 0),
                        Swerve.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig(false, false)),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, // Change this if the path needs to be flipped on red vs blue
                this); // Subsystem for requirements
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier =
                new Notifier(
                        () -> {
                            final double currentTime = Utils.getCurrentTimeSeconds();
                            double deltaTime = currentTime - m_lastSimTime;
                            m_lastSimTime = currentTime;

                            /* use the measured time delta, get battery voltage from WPILib */
                            updateSimState(deltaTime, RobotController.getBatteryVoltage());
                        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Rotation2d getHeading(){
        return this.getState().Pose.getRotation();
    }

    /*public Supplier<Rotation2d> getHeadingToSpeaker =
            () -> {
                Rotation2d aimGoal;

                if (RC.isRedAlliance.get()) { // Red side
                    if (Math.abs(this.getState().Pose.getY() - Field.kRedSpeakerPose2d.getY()) < 0.15) {
                        // aimGoal = new Rotation2d(Math.toRadians(-1 *
                        // this.getState().Pose.getRotation().getDegrees()));
                        aimGoal = new Rotation2d(0);
                    } else {
                        aimGoal =
                                new Rotation2d(
                                        Math.atan(
                                                (Field.kRedSpeakerPose2d.getY() - this.getState().Pose.getY())
                                                        / (Field.kRedSpeakerPose2d.getX() - this.getState().Pose.getX())));
                    }
                } else { // blue side
                    if (Math.abs(this.getState().Pose.getY() - Field.kBlueSpeakerPose2d.getY()) < 0.15) {
                        // aimGoal = new
                        // Rotation2d(Math.toRadians(this.getState().Pose.getRotation().getDegrees() -
                        // 180));
                        aimGoal = new Rotation2d(Math.PI);
                    } else {
                        aimGoal =
                                new Rotation2d(
                                        Math.atan(
                                                (Field.kBlueSpeakerPose2d.getY() - this.getState().Pose.getY())
                                                        / (Field.kBlueSpeakerPose2d.getX() - this.getState().Pose.getX()))
                                                + 180);
                    }
                }

                return aimGoal;
            };
     */

    public double normalize(double value, double maxVal, double minVal){
        return (value - minVal) / (maxVal - minVal);
    }

    public Supplier<Rotation2d> getHeadingToSpeaker =
            () -> {
                Rotation2d aimGoal;

                double x, y;
                boolean isRed = RC.isRedAlliance.get();

                if (Math.abs(this.getState().Pose.getY() - Field.kRedSpeakerPose2d.getY()) < 0.15){
                    aimGoal = isRed ? new Rotation2d(0) : new Rotation2d(Math.PI);
                }

                if (isRed) { // Red side
                    x = Field.kRedSpeakerPose2d.getX() - this.getState().Pose.getX();
                    y = Field.kRedSpeakerPose2d.getY() - this.getState().Pose.getY();
                    //aimGoal = new Rotation2d((Field.kRedSpeakerPose2d.getX() - this.getState().Pose.getX()), (Field.kRedSpeakerPose2d.getY() - this.getState().Pose.getY()));
                    //aimGoal = new Rotation2d(FastTrig.fastAtan2((float) y, (float) x));
                    aimGoal = new Rotation2d(Atan2LUT.getAtan2(x, y));
                } else { // blue side
                    x = Field.kBlueSpeakerPose2d.getX() - this.getState().Pose.getX();
                    y = Field.kBlueSpeakerPose2d.getY() - this.getState().Pose.getY();
                    //aimGoal = new Rotation2d((Field.kBlueSpeakerPose2d.getX() - this.getState().Pose.getX()), (Field.kBlueSpeakerPose2d.getY() - this.getState().Pose.getY()));
                    //aimGoal = new Rotation2d(FastTrig.fastAtan2((float) y, (float) x));
                    aimGoal = new Rotation2d(Atan2LUT.getAtan2(x, y));
                }

                return aimGoal;
            };

    public boolean inRangeToSpeaker(){
        Pose2d Speaker = RC.isRedAlliance.get() ? Field.kRedSpeakerPose2d : Field.kBlueSpeakerPose2d;
        Transform2d transform = this.getState().Pose.minus(Speaker);
        double distance = transform.getTranslation().getNorm();

        return distance > Swerve.disToRobot - Swerve.disToRobotError && distance < Swerve.disToRobot + Swerve.disToRobotError;
    }

    @Override
    public void periodic() {
        var PoseEstimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue(Swerve.Vision.kLimelightFrontName);
        if (PoseEstimate.tagCount >= 2) {
            this.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, Math.PI));
            this.addVisionMeasurement(PoseEstimate.pose, PoseEstimate.timestampSeconds);
        }

        ctrePublisher.set(this.getState().Pose);
        limelightPublisher.set(PoseEstimate.pose);

        Logger.recordOutput("Drivetrain/ctrePose", this.getState().Pose);
        Logger.recordOutput("Drivetrain/limelightPose", PoseEstimate.pose);

        /*
        SmartDashboard.putBoolean("2 tags", PoseEstimate.tagCount >= 2);
        SmartDashboard.putNumber("Turn angle", getHeadingToSpeaker.get().getDegrees());
        SmartDashboard.putNumber("Facing", this.getState().Pose.getRotation().getDegrees());
        SmartDashboard.putNumber("da", getHeadingToSpeaker.get().minus(this.getState().Pose.getRotation()).getDegrees());
        SmartDashboard.putBoolean("In Dinstance", inRangeToSpeaker());
        SmartDashboard.putNumber("tune-target", 0.00);
        SmartDashboard.putNumber("tune-angle", this.getState().Pose.getRotation().getDegrees());
         */
    }
}
