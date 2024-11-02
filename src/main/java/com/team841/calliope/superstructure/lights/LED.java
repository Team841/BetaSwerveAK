package com.team841.calliope.superstructure.lights;

import com.team841.calliope.Robot;
import com.team841.calliope.RobotContainer;
import com.team841.calliope.superstructure.indexer.Indexer;
import com.team841.calliope.superstructure.intake.Intake;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {

    private final LEDIO io;
    private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

    private Indexer indexer;

    private Intake intake;

    private int msCount = 0;

    /** Creates a new LED. */
    public LED(LEDIO io, Indexer indexer, Intake intake) {
       this.io = io;
       this.indexer = indexer;
       this.intake = intake;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LED", inputs);

        if (indexer.getRightIndexerSensor() && indexer.getLeftIndexerSensor()) {
            msCount = 0;
        }

        if(msCount < 3000){
            //green
            msCount += 20;
            io.setColor("green");
        }
        else {
            //red
            io.setColor("red");
        }
    }
}
