package com.team841.calliope.superstructure.hanger;

import org.littletonrobotics.junction.AutoLog;

public interface HangerIO {
    @AutoLog
    public class HangerIOInputs{
        public double velocity;
        public boolean flipped;
    }

    public default void updateInputs(HangerIOInputs inputs) { }

    public default void setVelocity(double Velocity) { }

    public default void stop() { }

    public default void flip() { }
}
