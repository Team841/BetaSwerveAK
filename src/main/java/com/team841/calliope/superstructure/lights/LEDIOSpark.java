package com.team841.calliope.superstructure.lights;

import com.team841.calliope.constants.SC;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDIOSpark implements LEDIO{
    private final Spark LED = new Spark(SC.Intake.kBlinkingID);

    public LEDIOSpark() {

    }

    @Override
    public void updateInputs(LedIOInputs inputs) {
       inputs.sparkOutput = LED.get();
    }

    @Override
    public void setColor(String color){
        switch (color) {
            case "Violet" -> LED.set(0.91);
            case "Green" -> LED.set(.77);
            case "Orange" -> LED.set(.65);
        }
    }

    @Override
    public void set(double value) {
        LED.set(value);
    }
}
