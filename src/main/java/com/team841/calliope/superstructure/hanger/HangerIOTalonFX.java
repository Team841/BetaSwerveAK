package com.team841.calliope.superstructure.hanger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team841.calliope.constants.RC;

public class HangerIOTalonFX implements HangerIO{
    private TalonFX LeftHangerMotor = new TalonFX(RC.SC_CAN_ID.kHangerMotorLeft, "rio");
    private TalonFX RightHangerMotor = new TalonFX(RC.SC_CAN_ID.kHangerMotorRight, "rio");

    private boolean flipped = true;

    public HangerIOTalonFX() {
        LeftHangerMotor.getConfigurator()
                .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        RightHangerMotor.getConfigurator()
                .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    }

    @Override
    public void updateInputs(HangerIOInputs inputs) {
        inputs.flipped = this.flipped;
        inputs.velocity = this.LeftHangerMotor.getVelocity().getValue();
    }

    @Override
    public void setVelocity(double velocity) {
        if (flipped){
            this.LeftHangerMotor.set(velocity);
            this.RightHangerMotor.set(velocity);
        } else {
            this.LeftHangerMotor.set(-velocity);
            this.LeftHangerMotor.set(-velocity);
        }
    }

    @Override
    public void stop() {
        this.LeftHangerMotor.stopMotor();
        this.RightHangerMotor.stopMotor();
    }

    @Override
    public void flip(){
        this.flipped = !this.flipped;
    }
}
