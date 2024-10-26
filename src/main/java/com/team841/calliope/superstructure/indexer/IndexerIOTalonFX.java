package com.team841.calliope.superstructure.indexer;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team841.calliope.constants.RC;
import com.team841.calliope.constants.SC;
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIOTalonFX implements IndexerIO{

    private final TalonFX indexerTalon = new TalonFX(RC.SC_CAN_ID.kIndexerTalon, "rio");

    private final DigitalInput rightSensor;
    private final DigitalInput leftSensor;

    private DutyCycleOut output = new DutyCycleOut(0);

    public IndexerIOTalonFX(){
        StatusCode indexStatus = indexerTalon.getConfigurator().apply(SC.Indexer.k_IndexerConfiguration);
        this.rightSensor = new DigitalInput(SC.Indexer.k_RightSensorChannel);
        this.leftSensor = new DigitalInput(SC.Indexer.k_LeftSensorChannel);

        if (!indexStatus.isOK())
        {
            RC.motorsAreWorking = false;
       }
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.leftSensor = !leftSensor.get();
        inputs.rightSensor = !rightSensor.get();
        inputs.velocity = this.indexerTalon.getVelocity().getValue();
    }

    @Override
    public void setDutyCycle(double velocity) {
        this.indexerTalon.setControl(output.withOutput(velocity));
    }

    @Override
    public void stopIndexer() {
        this.indexerTalon.stopMotor();
    }
}
