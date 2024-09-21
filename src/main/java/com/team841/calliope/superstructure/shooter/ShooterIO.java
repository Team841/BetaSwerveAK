package com.team841.calliope.superstructure.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs{
        public double velocity;
        public double motionMagicVoltageOutput;
    }

    public default void updateInputs(ShooterIOInputs inputs) { }

    public default void setMotionMagicVoltageOutput(double voltageOutput) { }

    public default void stopShooter() { }
}
