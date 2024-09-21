package com.team841.calliope.superstructure.indexer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team841.calliope.constants.RC;
import com.team841.calliope.constants.SC;
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIOTalonFX implements IndexerIO{

    private final TalonFX indexerTalon = new TalonFX(RC.SC_CAN_ID.kIndexerTalon, "rio");

    private DigitalInput rightSensor = new DigitalInput(SC.Indexer.k_RightSensorChannel);
    private DigitalInput leftSensor = new DigitalInput(SC.Indexer.k_LeftSensorChannel);

    private DutyCycleOut output = new DutyCycleOut(0);

    public IndexerIOTalonFX(){
        indexerTalon.getConfigurator().apply(SC.Indexer.k_IndexerConfiguration);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.leftSensor = !leftSensor.get();
        inputs.rightSensor = !rightSensor.get();
        inputs.velocity = this.indexerTalon.getVelocity().getValue();
    }

    public void setDutyCycle(double velocity) {
        this.indexerTalon.setControl(output.withOutput(velocity));
    }

    public void stopIndexer() {
        this.indexerTalon.stopMotor();
    }
}
