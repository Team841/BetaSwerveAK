package com.team841.calliope;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
import com.team841.calliope.superstructure.intake.*;
import com.team841.calliope.superstructure.lights.LED;
import com.team841.calliope.superstructure.lights.LEDIO;
import com.team841.calliope.superstructure.lights.LEDIOSpark;
import com.team841.calliope.superstructure.shooter.Shooter;
import com.team841.calliope.superstructure.shooter.ShooterIO;
import com.team841.calliope.superstructure.shooter.ShooterIOTalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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

    public final Feedback feedback;

    public final CommandXboxController sticksXbox[];

    public final CommandPS5Controller sticksPS5[];

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

    private SendableChooser<Command> autoChooser;

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

        registerNamedCommands();

        /*
        this.sticksXbox = new CommandXboxController[1];
        this.sticksPS5 = new CommandPS5Controller[1];
        this.sticksXbox[0] = new CommandXboxController(RC.Controllers.soloStick);

        this.bioDrive = new BioDrive(
                this.drivetrain,
                () -> -sticksXbox[0].getLeftY() * Swerve.MaxSpeed,
                () -> -sticksXbox[0].getLeftX() * Swerve.MaxSpeed,
                () -> -sticksXbox[0].getRightX() * Swerve.MaxAngularRate,
                () -> sticksXbox[0].a().getAsBoolean());

         */
                


        this.sticksPS5 = new CommandPS5Controller[1];
        this.sticksXbox = new CommandXboxController[1];
        this.sticksPS5[0] = new CommandPS5Controller(RC.Controllers.duoStickDrive);
        this.sticksXbox[0] = new CommandXboxController(RC.Controllers.duoStickCoDrive);

        this.bioDrive = new BioDrive(
                this.drivetrain,
                () -> -sticksPS5[0].getLeftY() * Swerve.MaxSpeed,
                () -> -sticksPS5[0].getLeftX() * Swerve.MaxSpeed,
                () -> -sticksPS5[0].getRightX() * Swerve.MaxAngularRate,
                () -> sticksPS5[0].L2().getAsBoolean(),
                () -> sticksXbox[0].rightBumper().getAsBoolean(),
                () -> -sticksXbox[0].getRightX() * 2 * Math.PI);



        this.feedback = new Feedback(this.sticksXbox[0]);

        this.shootCommand = new Shoot(this.indexer, this.shooter);


        /*this.bioDrive = new BioDrive(
                this.drivetrain,
                () -> -sticksXbox[0].getLeftY() * Swerve.MaxSpeed,
                () -> -sticksXbox[0].getLeftX() * Swerve.MaxSpeed,
                () -> -sticksXbox[0].getRightX() * Swerve.MaxAngularRate,
                () -> sticksXbox[0].a().getAsBoolean());

         */

        this.drivetrain.setDefaultCommand(bioDrive);

        configureDuoStick();
        //configureSoloStick();

    }


    private void configureSoloStick() {
        /*this.drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () ->
                                drive
                                        .withVelocityX(-sticksXbox[0].getLeftY() * Swerve.MaxSpeed) // Drive forward with
                                        // negative Y (forward)
                                        .withVelocityY(
                                                -sticksXbox[0].getLeftX() * Swerve.MaxSpeed) // Drive left with negative X (left)
                                        .withRotationalRate(
                                                -sticksXbox[0].getRightX()
                                                        * Swerve
                                                        .MaxAngularRate))); // Drive counterclockwise with negative X (left)

         */

        // reset the field-centric heading on left bumper press
        sticksXbox[0].start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        } else {
            drivetrain.registerTelemetry(telemetry::telemeterize);
        }

        Command c_command = new IntakeCommand(intake, indexer);
        sticksXbox[0].leftBumper().whileTrue(c_command);
        sticksXbox[0]
                .leftTrigger()
                .onTrue(new InstantCommand(shooter::spinUp))
                .onFalse(new InstantCommand(shooter::stopShooter));
        sticksXbox[0]
                .rightTrigger()
                .onTrue(
                        new ConditionalCommand(
                                new InstantCommand(indexer::Pass),
                                new InstantCommand(indexer::stopIndexer),
                                () -> shooter.isShooting()))
                .onFalse(new InstantCommand(indexer::stopIndexer));
        sticksXbox[0]
                .x()
                .onTrue(
                        new SequentialCommandGroup(
                                new InstantCommand(indexer::stopIndexer),
                                new InstantCommand(shooter::stopShooter)));
        sticksXbox[0].rightStick().whileTrue(new InstantCommand(hanger::ExtendHanger)).onFalse(new InstantCommand(hanger::StopHanger));
        sticksXbox[0].leftStick().whileTrue(new InstantCommand(hanger::RetractHanger)).onFalse(new InstantCommand(hanger::StopHanger));
        sticksXbox[0].povCenter().whileTrue(new InstantCommand(hanger::StopHanger));
        sticksXbox[0].povLeft().whileTrue(new InstantCommand(hanger::toggleHanger));
        sticksXbox[0]
                .y()
                .onTrue(new InstantCommand(shooter::ampShot))
                .onFalse(new InstantCommand(shooter::stopShooter));
        sticksXbox[0]
                .b()
                .onTrue(
                        new ParallelCommandGroup(
                                new InstantCommand(intake::outTake), new InstantCommand(indexer::reverseIndexer)))
                .onFalse(
                        new SequentialCommandGroup(
                                new InstantCommand(indexer::stopIndexer), new InstantCommand(intake::stopIntake)));

        sticksXbox[0]
                .rightBumper()
                .whileTrue(new InstantCommand(shooter::flyShot))
                .onFalse(new InstantCommand(shooter::stopShooter));
    }
    private void configureDuoStick() {
        /*drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () ->
                                drive
                                        .withVelocityX(-sticksPS5[0].getLeftY() * Swerve.MaxSpeed) // Drive forward with
                                        // negative Y (forward)
                                        .withVelocityY(
                                                -sticksPS5[0].getLeftX() * Swerve.MaxSpeed) // Drive left with negative X (left)
                                        .withRotationalRate(
                                                -sticksPS5[0].getRightX()
                                                        * Swerve
                                                        .MaxAngularRate))); // Drive counterclockwise with negative X (left)

        */


        sticksPS5[0].cross().whileTrue(drivetrain.applyRequest(() -> brake));
        sticksPS5[0]
                .circle()
                .whileTrue(
                        drivetrain.applyRequest(
                                () ->
                                        point.withModuleDirection(
                                                new Rotation2d(-sticksPS5[0].getLeftY(), -sticksPS5[0].getLeftX()))));

        // reset the field-centric heading on left bumper press
        sticksPS5[0].touchpad().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        //sticksPS5[0].R2().whileTrue(autoAim);

        //sticksPS5[0].triangle().whileTrue(BackOffTrap);

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        } else {
            drivetrain.registerTelemetry(telemetry::telemeterize);
        }

        Command c_command = new IntakeCommand(intake, indexer);
        sticksXbox[0].leftBumper().whileTrue(c_command);
        sticksXbox[0]
                .leftTrigger()
                .onTrue(new InstantCommand(shooter::spinUp))
                .onFalse(new InstantCommand(shooter::stopShooter));
        sticksXbox[0]
                .rightTrigger()
                .onTrue(
                        new ConditionalCommand(
                                new InstantCommand(indexer::Pass),
                                new InstantCommand(indexer::stopIndexer),
                                () -> shooter.isShooting()))
                .onFalse(new InstantCommand(indexer::stopIndexer));
        sticksXbox[0]
                .start()
                .onTrue(
                        new SequentialCommandGroup(
                                new InstantCommand(indexer::stopIndexer),
                                new InstantCommand(shooter::stopShooter)));
        sticksXbox[0].povUp().whileTrue(new InstantCommand(hanger::ExtendHanger)).onFalse(new InstantCommand(hanger::StopHanger));
        sticksXbox[0].povDown().whileTrue(new InstantCommand(hanger::RetractHanger)).onFalse(new InstantCommand(hanger::StopHanger));
        sticksXbox[0].povCenter().whileTrue(new InstantCommand(hanger::StopHanger));
        sticksXbox[0].povLeft().whileTrue(new InstantCommand(hanger::toggleHanger));
        sticksXbox[0]
                .x()
                .onTrue(new InstantCommand(shooter::ampShot))
                .onFalse(new InstantCommand(shooter::stopShooter));
        sticksXbox[0]
                .b()
                .onTrue(
                        new ParallelCommandGroup(
                                new InstantCommand(intake::outTake), new InstantCommand(indexer::reverseIndexer)))
                .onFalse(
                        new SequentialCommandGroup(
                                new InstantCommand(indexer::stopIndexer), new InstantCommand(intake::stopIntake)));

        sticksXbox[0]
                .y()
                .whileTrue(new InstantCommand(shooter::flyShot))
                .onFalse(new InstantCommand(shooter::stopShooter));
        sticksXbox[0]
                .a()
                .onTrue(new InstantCommand(shooter::trapShot))
                .onFalse(new InstantCommand(shooter::stopShooter));
    }

    private void registerNamedCommands(){
        // Register Named Commands
        NamedCommands.registerCommand("IntakeOn", new IntakeAuto(intake, indexer));
        NamedCommands.registerCommand(
                "Shoot",
                new ParallelCommandGroup(
                        new InstantCommand(shooter::spinUp),
                        new SequentialCommandGroup(new WaitCommand(1), new InstantCommand(indexer::Pass)))
                        .withTimeout(3));
        NamedCommands.registerCommand("SpinUp", new InstantCommand(shooter::spinUp));
        NamedCommands.registerCommand("JustShoot", new InstantCommand(indexer::Pass).withTimeout(0.5));
        NamedCommands.registerCommand(
                "ALLSYSTEMSGO",
                new ParallelCommandGroup(
                        new InstantCommand(intake::intake),
                        new InstantCommand(shooter::disruptshot),
                        new InstantCommand(indexer::Pass)));
        NamedCommands.registerCommand(
                "FunnyInake",
                new ParallelCommandGroup(
                        new InstantCommand(intake::intake), new InstantCommand(indexer::Pass))
                        .withTimeout(0.75));
        NamedCommands.registerCommand(
                "JustStop",
                new ParallelCommandGroup(
                        new InstantCommand(indexer::stopIndexer), new InstantCommand(shooter::stopShooter)));
        NamedCommands.registerCommand(
                "sam",
                new ParallelCommandGroup(
                        new InstantCommand(intake::intake),
                        new InstantCommand(shooter::ampShot),
                        new InstantCommand(indexer::Pass))
                        .withTimeout(2.5));
        NamedCommands.registerCommand("aIntake", new IntakeAuto(intake, indexer));
        NamedCommands.registerCommand("stopIndexer", new InstantCommand(indexer::stopIndexer));
        NamedCommands.registerCommand(
                "CountShot",
                new ParallelCommandGroup(
                        new InstantCommand(shooter::spinUp),
                        new SequentialCommandGroup(new WaitCommand(1), new InstantCommand(indexer::Pass)))
                        .withTimeout(0.8));
        NamedCommands.registerCommand("STOPALL", new ParallelCommandGroup(
                new InstantCommand(indexer::stopIndexer), new InstantCommand(shooter::stopShooter), new InstantCommand(intake::stopIntake)));

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
