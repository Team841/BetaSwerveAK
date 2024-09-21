package com.team841.calliope.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team841.calliope.vision.LimelightHelpers;
import com.team841.calliope.constants.Field;
import com.team841.calliope.constants.RC;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com .team841.calliope.constants.Swerve;

import java.util.function.Supplier;

public class Drivetrain extends SwerveDrivetrain implements Subsystem {

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  private final SwerveRequest.ApplyChassisSpeeds autoRequest =
      new SwerveRequest.ApplyChassisSpeeds();

  public Drivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }

    this.setOperatorPerspectiveForward(
        RC.isRedAlliance.get() ? new Rotation2d(Math.PI) : new Rotation2d(0.0));

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
        RC.isRedAlliance.get() ? new Rotation2d(Math.PI) : new Rotation2d(0.0));

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

  public void seedTemp() {
    this.seedFieldRelative(
        GeometryUtil.flipFieldPose(
            new Pose2d(1.3865381479263306, 4.631037712097168, new Rotation2d(3.14159))));
  }

  public LimelightHelpers.PoseEstimate getLimeLightPoses() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(Swerve.Vision.kLimelightFrontName);
  }

  public Supplier<Rotation2d> getHeadingToSpeaker =
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

  protected Supplier<Pose2d> ComputeThreadGetPose =
      () -> {
        return this.getState().Pose;
      };

  @Override
  public void periodic() {
  }
}
