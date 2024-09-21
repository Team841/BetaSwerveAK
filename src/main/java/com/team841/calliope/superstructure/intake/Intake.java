package com.team841.calliope.superstructure.intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setIntakeBrakes(boolean on) {
        io.setBrakes(on);
    }

    public void intake() {
        io.setVelocity(0.75);
    }

    public void outTake() {
        io.setVelocity(-1.0);
    }

    public void stopIntake() {
        io.stopIntake();
    }
}
