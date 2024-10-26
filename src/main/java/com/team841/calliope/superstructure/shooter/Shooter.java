package com.team841.calliope.superstructure.shooter;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public void spinUp() {
        io.setMotionMagicVelocityVoltageOutput(95);
    }

    public void ampShot() {
        io.setMotionMagicVelocityVoltageOutput(1.5, 12.5);
    }

    public Command idleBack() {
        return new RunCommand(
                () -> {
                    io.setMotionMagicVelocityVoltageOutput(-0.015, -0.015);
                },
                this);
    }

    public void trapShot() {
        double top = 9.9375;
        double bottom = 82.8125;
        io.setMotionMagicVelocityVoltageOutput(top, bottom);
    }

    public void flyShot() {
        io.setMotionMagicVelocityVoltageOutput(70);
    }

    public void disruptshot() {
        io.setMotionMagicVelocityVoltageOutput(12.5);
    }

    public boolean isShooting() {
        return this.inputs.topVelocity > 0;
    }

    public boolean isHighShot() {
        return this.inputs.topVelocity > 50;
    }

    public void stopShooter() {
        io.setMotionMagicVelocityVoltageOutput(-0.015);
    }

    public Command runShooter(double velocity) {
        return new InstantCommand(() -> io.setMotionMagicVelocityVoltageOutput(velocity));
    }
}
