package com.team841.calliope.superstructure.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team841.calliope.constants.RC;
import com.team841.calliope.constants.SC;

public class ShooterIOTalonFX implements ShooterIO{
    private final TalonFX bottomShooter = new TalonFX(RC.SC_CAN_ID.bottomShooter, "rio");
    private final TalonFX topShooter = new TalonFX(RC.SC_CAN_ID.topShooter, "rio");

    public ShooterIOTalonFX() {
        bottomShooter.getConfigurator().apply(SC.Shooter.k_BottomConfiguration);
        topShooter.getConfigurator().apply(SC.Shooter.k_TopConfiguration);
    }

    public void updateInputs(ShooterIOInputs inputs) {
        inputs.velocity = bottomShooter
    }

    public default void setMotionMagicVoltageOutput(double voltageOutput) { }

    public default void stopShooter() { }
}
