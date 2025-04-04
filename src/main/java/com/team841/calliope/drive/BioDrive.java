package com.team841.calliope.drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.team841.calliope.constants.Swerve;
import com.team841.calliope.superstructure.Shoot;
import com.team841.lib.util.LoggedTunableNumber;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class BioDrive extends Command {

    /*public BioDrive(Drivetrain drivetrain, DoubleSupplier velocityXGetter, DoubleSupplier velocityYGetter, DoubleSupplier velocityOmegaGetter, BooleanSupplier faceSpeakerGetter, BooleanSupplier autoShootGetter, Command shootCommand) {
        this(drivetrain, velocityXGetter, velocityYGetter, velocityOmegaGetter, faceSpeakerGetter);
        this.shootCommand = shootCommand;
        this.mAutoShoot = autoShootGetter;
    }

     */


    public BioDrive(Drivetrain drivetrain, DoubleSupplier velocityXGetter, DoubleSupplier velocityYGetter, DoubleSupplier velocityOmegaGetter, BooleanSupplier faceSpeakerGetter) {

        this.drivetrain = drivetrain;

        this.fieldCentricFacingAngle.HeadingController.setPID(Swerve.HeadingController.kp, Swerve.HeadingController.ki, Swerve.HeadingController.kd);
        this.fieldCentricFacingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        this.fieldCentricFacingAngle.HeadingController.setTolerance(0.0000005);

        this.mVelocityX = velocityXGetter;
        this.mVelocityY = velocityYGetter;
        this.mVelocityOmega = velocityOmegaGetter;
        this.mFaceSpeaker = faceSpeakerGetter;

        addRequirements(drivetrain);
        setName("BioDrive");

        Swerve.controller = this.fieldCentricFacingAngle.HeadingController;
    }

    private Command shootCommand;

    private DoubleSupplier mVelocityX, mVelocityY, mVelocityOmega;
    private BooleanSupplier mFaceSpeaker, mAutoShoot;

    private boolean faceSpeaker = false;
    private double velocity_y = 0.0;
    private double velocity_x = 0.0;
    private double velocity_omega = 0.0;
    private boolean autoShoot = false;

    private Drivetrain drivetrain;

    private final SwerveRequest.FieldCentric fieldCentricDrive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(Swerve.MaxSpeed * 0.1)
                    .withRotationalDeadband(Swerve.MaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withRotationalDeadband(Swerve.MaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

    @Override
    public void initialize() {
        return;
    }

    @Override
    public void execute() {

        this.faceSpeaker = mFaceSpeaker.getAsBoolean();
        this.velocity_x = mVelocityX.getAsDouble();
        this.velocity_y = mVelocityY.getAsDouble();
        this.velocity_omega = mVelocityOmega.getAsDouble();
        //this.autoShoot = mAutoShoot.getAsBoolean();

        Logger.recordOutput("BioDrive/FaceSpeaker", this.faceSpeaker);
        Logger.recordOutput("BioDrive/velocityX", this.velocity_x);
        Logger.recordOutput("BioDrive/velocityY", this.velocity_y);
        Logger.recordOutput("BioDrive/velocityOmega", this.velocity_omega);

        if (this.faceSpeaker) {
            Rotation2d angleToSpeaker = drivetrain.getHeadingToSpeaker.get();
            Logger.recordOutput("BioDrive/headingToSpeaker", angleToSpeaker);

            this.drivetrain.setControl(fieldCentricFacingAngle.withVelocityX(-this.velocity_x).withVelocityY(-this.velocity_y).withTargetDirection(angleToSpeaker));

            /*Rotation2d angle = new Rotation2d(0);
            this.drivetrain.setControl(fieldCentricFacingAngle.withVelocityX(-this.velocity_x).withVelocityY(-this.velocity_y).withTargetDirection(angle));

             */
        } else {
            this.drivetrain.setControl(fieldCentricDrive.withVelocityX(-this.velocity_x).withVelocityY(-this.velocity_y).withRotationalRate(this.velocity_omega));
        }

        /*if (this.mAutoShoot != null) {
            if (this.autoShoot && this.drivetrain.inRangeToSpeaker() && !CommandScheduler.getInstance().isScheduled(this.shootCommand)) {
                CommandScheduler.getInstance().schedule(this.shootCommand);
            }
        }

         */
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ended");
    }
}
