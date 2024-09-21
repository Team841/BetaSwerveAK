// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team841.calliope.superstructure.hanger;

import com.team841.calliope.constants.RC;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hanger extends SubsystemBase {

    private final HangerIO io;
    private final HangerIOInputsAutoLogged inputs = new HangerIOInputsAutoLogged();

    /** Creates a new Hanger. */
    public Hanger(HangerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs); // Update input data from the IO layer
        Logger.processInputs("Hanger", inputs); // Send input data to the logging framework (or update from the log during replay)
    }

    public void ExtendHanger() {
        io.setVelocity(1.00);
    }

    public void RetractHanger() {
        io.setVelocity(0.98);
    }

    public void toggleHanger() {
        io.flip();
    }

    public void StopHanger() {
        io.stop();
    }
}
