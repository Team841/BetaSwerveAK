package com.team841.calliope;

import com.team841.calliope.constants.RC;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Feedback {

    private int count = 0;

    private CommandXboxController controller;

    public Feedback(CommandXboxController controller) {
        this.controller = controller;
    }

    public void Intaked() {
        RC.rumbleNeedsPing = true;
        controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);
        count += 1;
    }

    public void update() {
        if (count == 100) {
            RC.rumbleNeedsPing = false;
            controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
            count = 0;
        } else {
            count += 1;
        }
    }

    public boolean nowCounting() {
        return count != 0;
    }
}
