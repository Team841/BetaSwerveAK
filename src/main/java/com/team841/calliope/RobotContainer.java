package com.team841.calliope;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.team841.calliope.constants.RC;
import com.team841.calliope.constants.Swerve;
import com.team841.calliope.drive.BioDrive;
import com.team841.calliope.drive.Drivetrain;
import com.team841.calliope.superstructure.Shoot;
import com.team841.calliope.superstructure.hanger.Hanger;
import com.team841.calliope.superstructure.hanger.HangerIO;
import com.team841.calliope.superstructure.hanger.HangerIOTalonFX;
import com.team841.calliope.superstructure.indexer.Indexer;
import com.team841.calliope.superstructure.indexer.IndexerIO;
import com.team841.calliope.superstructure.indexer.IndexerIOTalonFX;
import com.team841.calliope.superstructure.intake.Intake;
import com.team841.calliope.superstructure.intake.IntakeCommand;
import com.team841.calliope.superstructure.intake.IntakeIO;
import com.team841.calliope.superstructure.intake.IntakeIOTalonFX;
import com.team841.calliope.superstructure.lights.LED;
import com.team841.calliope.superstructure.lights.LEDIO;
import com.team841.calliope.superstructure.lights.LEDIOSpark;
import com.team841.calliope.superstructure.shooter.Shooter;
import com.team841.calliope.superstructure.shooter.ShooterIO;
import com.team841.calliope.superstructure.shooter.ShooterIOTalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static com.team841.calliope.constants.Swerve.DrivetrainConstants;

public class RobotContainer {

    public final Drivetrain drivetrain;

    public final ShooterIO shooterIO;
    public final Shooter shooter;

    public final IntakeIO intakeIO;
    public final Intake intake;

    public final IndexerIO indexerIO;
    public final Indexer indexer;

    public final HangerIO hangerIO;
    public final Hanger hanger;

    public final LEDIO ledIO;
    public final LED led;

    //public final CommandXboxController soloStick = new CommandXboxController(RC.Controllers.soloStick);

    public final CommandPS5Controller duoStickDrive = new CommandPS5Controller(RC.Controllers.duoStickDrive);
    public final CommandXboxController duoStickCoDrive = new CommandXboxController(RC.Controllers.duoStickCoDrive);

    /*private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(Swerve.MaxSpeed * 0.1)
                    .withRotationalDeadband(Swerve.MaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

     */

    // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry telemetry = new Telemetry(Swerve.MaxSpeed);

    private BioDrive bioDrive;
    private Shoot shootCommand;

    private static volatile RobotContainer instance;

    public static RobotContainer getInstance() {
        if (instance == null) {
            synchronized (RobotContainer.class) {
                if (instance == null) {
                    instance = new RobotContainer();
                }
            }
        }
        return instance;
    }

    public RobotContainer() {
        switch (RC.robotType) {
            default -> {
                this.drivetrain = new Drivetrain(Swerve.DrivetrainConstants, Swerve.FrontLeft, Swerve.FrontRight, Swerve.BackLeft, Swerve.BackRight);

                this.shooterIO = new ShooterIOTalonFX();
                this.shooter = new Shooter(this.shooterIO);

                this.intakeIO = new IntakeIOTalonFX();
                this.intake = new Intake(this.intakeIO);

                this.indexerIO = new IndexerIOTalonFX();
                this.indexer = new Indexer(this.indexerIO);

                this.hangerIO = new HangerIOTalonFX();
                this.hanger = new Hanger(this.hangerIO);

                this.ledIO = new LEDIOSpark();
                this.led = new LED(this.ledIO, this.indexer, this.intake);
            }
        }

        this.shootCommand = new Shoot(this.indexer, this.shooter);

        this.bioDrive = new BioDrive(
                this.drivetrain,
                () -> -duoStickDrive.getLeftY() * Swerve.MaxSpeed,
                () -> -duoStickDrive.getLeftX() * Swerve.MaxSpeed,
                () -> -duoStickDrive.getRightX() * Swerve.MaxAngularRate,
                () -> duoStickDrive.L2().getAsBoolean(),
                ()->duoStickDrive.R2().getAsBoolean(),
                shootCommand);


        /*this.bioDrive = new BioDrive(
                this.drivetrain,
                () -> -soloStick.getLeftY() * Swerve.MaxSpeed,
                () -> -soloStick.getLeftX() * Swerve.MaxSpeed,
                () -> -soloStick.getRightX() * Swerve.MaxAngularRate,
                () -> soloStick.a().getAsBoolean());

         */

        this.drivetrain.setDefaultCommand(bioDrive);

        //configureSoloStick();
        configureDuoStick();
    }


    //private void configureSoloStick() {
        /*this.drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () ->
                                drive
                                        .withVelocityX(-soloStick.getLeftY() * Swerve.MaxSpeed) // Drive forward with
                                        // negative Y (forward)
                                        .withVelocityY(
                                                -soloStick.getLeftX() * Swerve.MaxSpeed) // Drive left with negative X (left)
                                        .withRotationalRate(
                                                -soloStick.getRightX()
                                                        * Swerve
                                                        .MaxAngularRate))); // Drive counterclockwise with negative X (left)

         */

    /*
        // reset the field-centric heading on left bumper press
        soloStick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        } else {
            drivetrain.registerTelemetry(telemetry::telemeterize);
        }

        Command c_command = new IntakeCommand(intake, indexer);
        soloStick.leftBumper().whileTrue(c_command);
        soloStick
                .leftTrigger()
                .onTrue(new InstantCommand(shooter::spinUp))
                .onFalse(new InstantCommand(shooter::stopShooter));
        soloStick
                .rightTrigger()
                .onTrue(
                        new ConditionalCommand(
                                new InstantCommand(indexer::Pass),
                                new InstantCommand(indexer::stopIndexer),
                                () -> shooter.isShooting()))
                .onFalse(new InstantCommand(indexer::stopIndexer));
        soloStick
                .x()
                .onTrue(
                        new SequentialCommandGroup(
                                new InstantCommand(indexer::stopIndexer),
                                new InstantCommand(shooter::stopShooter)));
        //soloStick.rightStick().whileTrue(new InstantCommand(hanger::ExtendHanger)).onFalse(new InstantCommand(hanger::StopHanger));
        //soloStick.leftStick().whileTrue(new InstantCommand(hanger::RetractHanger)).onFalse(new InstantCommand(hanger::StopHanger));
        soloStick.povCenter().whileTrue(new InstantCommand(hanger::StopHanger));
        soloStick.povLeft().whileTrue(new InstantCommand(hanger::toggleHanger));
        soloStick
                .y()
                .onTrue(new InstantCommand(shooter::ampShot))
                .onFalse(new InstantCommand(shooter::stopShooter));
        soloStick
                .b()
                .onTrue(
                        new ParallelCommandGroup(
                                new InstantCommand(intake::outTake), new InstantCommand(indexer::reverseIndexer)))
                .onFalse(
                        new SequentialCommandGroup(
                                new InstantCommand(indexer::stopIndexer), new InstantCommand(intake::stopIntake)));

        soloStick
                .rightBumper()
                .whileTrue(new InstantCommand(shooter::flyShot))
                .onFalse(new InstantCommand(shooter::stopShooter));
    }

     */


    private void configureDuoStick() {
        /*drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () ->
                                drive
                                        .withVelocityX(-duoStickDrive.getLeftY() * Swerve.MaxSpeed) // Drive forward with
                                        // negative Y (forward)
                                        .withVelocityY(
                                                -duoStickDrive.getLeftX() * Swerve.MaxSpeed) // Drive left with negative X (left)
                                        .withRotationalRate(
                                                -duoStickDrive.getRightX()
                                                        * Swerve
                                                        .MaxAngularRate))); // Drive counterclockwise with negative X (left)

        */


        duoStickDrive.cross().whileTrue(drivetrain.applyRequest(() -> brake));
        duoStickDrive
                .circle()
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        point.withModuleDirection(
                                                new Rotation2d(-duoStickDrive.getLeftY(), -duoStickDrive.getLeftX()))));

        // reset the field-centric heading on left bumper press
        duoStickDrive.touchpad().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        //duoStickDrive.R2().whileTrue(autoAim);

        //duoStickDrive.triangle().whileTrue(BackOffTrap);

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        } else {
            drivetrain.registerTelemetry(telemetry::telemeterize);
        }

        Command c_command = new IntakeCommand(intake, indexer);
        duoStickCoDrive.leftBumper().whileTrue(c_command);
        duoStickCoDrive
                .leftTrigger()
                .onTrue(new InstantCommand(shooter::spinUp))
                .onFalse(new InstantCommand(shooter::stopShooter));
        duoStickCoDrive
                .rightTrigger()
                .onTrue(
                        new ConditionalCommand(
                                new InstantCommand(indexer::Pass),
                                new InstantCommand(indexer::stopIndexer),
                                () -> shooter.isShooting()))
                .onFalse(new InstantCommand(indexer::stopIndexer));
        duoStickCoDrive
                .rightBumper()
                .onTrue(
                        new SequentialCommandGroup(
                                new InstantCommand(indexer::stopIndexer),
                                new InstantCommand(shooter::stopShooter)));
        //duoStickCoDrive.povUp().whileTrue(new InstantCommand(hanger::ExtendHanger));
        //duoStickCoDrive.povDown().whileTrue(new InstantCommand(hanger::RetractHanger));
        duoStickCoDrive.povCenter().whileTrue(new InstantCommand(hanger::StopHanger));
        duoStickCoDrive.povLeft().whileTrue(new InstantCommand(hanger::toggleHanger));
        duoStickCoDrive
                .x()
                .onTrue(new InstantCommand(shooter::ampShot))
                .onFalse(new InstantCommand(shooter::stopShooter));
        duoStickCoDrive
                .b()
                .onTrue(
                        new ParallelCommandGroup(
                                new InstantCommand(intake::outTake), new InstantCommand(indexer::reverseIndexer)))
                .onFalse(
                        new SequentialCommandGroup(
                                new InstantCommand(indexer::stopIndexer), new InstantCommand(intake::stopIntake)));

        duoStickCoDrive
                .y()
                .whileTrue(new InstantCommand(shooter::flyShot))
                .onFalse(new InstantCommand(shooter::stopShooter));
        duoStickCoDrive
                .a()
                .onTrue(new InstantCommand(shooter::trapShot))
                .onFalse(new InstantCommand(shooter::stopShooter));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
