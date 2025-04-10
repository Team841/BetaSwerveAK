package com.team841.calliope.superstructure.shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team841.calliope.constants.RC;
import com.team841.calliope.constants.SC;

public class ShooterIOTalonFX implements ShooterIO{
    private final TalonFX bottomShooter = new TalonFX(RC.SC_CAN_ID.bottomShooter, "rio");
    private final TalonFX topShooter = new TalonFX(RC.SC_CAN_ID.topShooter, "rio");

    private final MotionMagicVelocityVoltage topControl = new MotionMagicVelocityVoltage(0).withFeedForward(6).withAcceleration(200).withSlot(0);
    private final MotionMagicVelocityVoltage bottomControl = new MotionMagicVelocityVoltage(0).withFeedForward(6).withAcceleration(200).withSlot(0);

    public ShooterIOTalonFX() {
        StatusCode bShooterStatus = bottomShooter.getConfigurator().apply(SC.Shooter.k_BottomConfiguration);
        StatusCode tShooterStatus = topShooter.getConfigurator().apply(SC.Shooter.k_TopConfiguration);

        if (!bShooterStatus.isOK() || !tShooterStatus.isOK())
        {
            RC.motorsAreWorking = false;
        }
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topVelocity = topShooter.getVelocity().getValue();
        inputs.bottomVelocity = bottomShooter.getVelocity().getValue();
        inputs.bottomMotionMagicVoltageVelocity = bottomControl.Velocity;
        inputs.topMotionMagicVoltageVelocity = topControl.Velocity;
        inputs.dutyCycleOut = topShooter.getDutyCycle().getValue();
    }

    @Override
    public void setMotionMagicVelocityVoltageOutput(double velocity) {
        bottomShooter.setControl(bottomControl.withVelocity(velocity));
        topShooter.setControl(topControl.withVelocity(velocity));
    }

    @Override
    public void setMotionMagicVelocityVoltageOutput(double topVelocity, double bottomVelocity) {
        bottomShooter.setControl(bottomControl.withVelocity(bottomVelocity));
        topShooter.setControl(topControl.withVelocity(topVelocity));
    }

    @Override
    public void setDutyCycle(double value){
        topShooter.set(value);
        bottomShooter.set(value);
    }

    @Override
    public void stop() {
        bottomShooter.stopMotor();
        topShooter.stopMotor();
    }
}
