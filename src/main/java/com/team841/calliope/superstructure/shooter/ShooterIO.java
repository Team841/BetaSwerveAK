package com.team841.calliope.superstructure.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs{
        public double topVelocity;
        public double bottomVelocity;
        public double topMotionMagicVoltageVelocity;
        public double bottomMotionMagicVoltageVelocity;
        public double dutyCycleOut;

    }

    public default void updateInputs(ShooterIOInputs inputs) { }

    public default void setDutyCycle(double value) { }

    public default void setMotionMagicVelocityVoltageOutput(double velocity) { }

    public default void setMotionMagicVelocityVoltageOutput(double topVelocity, double bottomVelocity) { }

    public default void stop() { }
}
