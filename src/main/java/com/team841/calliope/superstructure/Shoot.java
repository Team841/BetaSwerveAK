package com.team841.calliope.superstructure;

import com.team841.calliope.superstructure.indexer.Indexer;
import com.team841.calliope.superstructure.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class Shoot extends Command {

    private Indexer mIndexer;
    private Shooter mShooter;
    double passStartTime = -1;

    public Shoot(Indexer indexer, Shooter shooter) {
        this.mIndexer = indexer;
        this.mShooter = shooter;
    }

    @Override
    public void initialize(){
        this.mShooter.spinUp();
    }

    @Override
    public void execute(){
        if (this.mShooter.isHighShot()){
            this.mIndexer.FULLPASS();
            if (this.passStartTime == -1)
                this.passStartTime = Logger.getRealTimestamp();
        }
    }

    @Override
    public boolean isFinished() {
        return this.passStartTime != -1 && Logger.getRealTimestamp() - passStartTime > 1.5;
    }

    @Override
    public void end(boolean interrupted) {
        this.mIndexer.stopIndexer();
        this.mShooter.stopShooter();
        this.passStartTime = -1;
    }
}
