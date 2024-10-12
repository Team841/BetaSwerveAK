package com.team841.calliope.superstructure.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs{
        public double velocity;
        public boolean brakesOn;
    }

    public default void updateInputs(IntakeIOInputs inputs) { }

    public default void setBrakes(boolean on) { }

    public default void setVelocity(double velocity) { }

    public default void stopIntake() { }
}
