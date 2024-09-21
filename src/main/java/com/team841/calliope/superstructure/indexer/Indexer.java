package com.team841.calliope.superstructure.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();


    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }

    public void intake() {
        io.setDutyCycle(-0.6);
    }

    public void halfIntake() {
        io.setDutyCycle(-0.3);
    }

    public void quarterIntake() {
        io.setDutyCycle(-0.15);
    }

    public void stopIndexer() {
        io.stopIndexer();
    }

    public void Pass() {
        io.setDutyCycle(-0.6);
    }

    public boolean getRightIndexerSensor() {
        return inputs.rightSensor;
    }

    public boolean getLeftIndexerSensor() {
        return inputs.leftSensor;
    }

    public void reverseIndexer() {
        io.setDutyCycle(0.6);
    }
}
