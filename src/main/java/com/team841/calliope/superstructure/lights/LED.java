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

    private int count = 0;

    /** Creates a new LED. */
    public LED(LEDIO io) {
       this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LED", inputs);

        if (this.indexer == null)
            this.indexer = Robot.m_robotContainer.indexer;
        if (this.intake == null)
            this.intake = Robot.m_robotContainer.intake;

        // if (intake.)
        if (indexer.getRightIndexerSensor() && indexer.getLeftIndexerSensor()) {
            io.setColor("Green");
            if (count == 0) count += 1;
        }

        if (count > 0) {
            count += 1;
        }

        if (count > 200) {
            io.setColor("Violet");
            count = 0;
        }
    }
}
