package com.team841.calliope.superstructure.intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team841.calliope.constants.RC;

public class IntakeIOTalonFX {

    private final TalonFX intakeMotor = new TalonFX(RC.SC_CAN_ID.kIntake, "rio");

    private boolean brakesOn = true;

    public IntakeIOTalonFX(){
       setBrakes(brakesOn);
    }

    public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
        inputs.brakesOn = brakesOn;
        inputs.velocity = this.intakeMotor.getVelocity().getValue();
    }

    public void setBrakes(boolean on) {
       intakeMotor
                .getConfigurator()
                .apply(
                        new MotorOutputConfigs()
                                .withNeutralMode(on ? NeutralModeValue.Brake : NeutralModeValue.Coast));
    }

    public void setVelocity(double velocity) {
        intakeMotor.set(velocity);
    }

    public void stopIntake() {
        intakeMotor.stopMotor();
    }

}
