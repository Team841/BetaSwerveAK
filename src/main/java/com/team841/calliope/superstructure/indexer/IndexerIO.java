package com.team841.calliope.superstructure.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public class IndexerIOInputs{
        public boolean leftSensor;
        public boolean rightSensor;
        public double velocity;
    }

    public default void updateInputs(IndexerIOInputs inputs) { }

    public default void setDutyCycle(double velocity) { }

    public default void stopIndexer() { }

}
