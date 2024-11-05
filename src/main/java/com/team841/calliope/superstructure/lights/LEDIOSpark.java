package com.team841.calliope.superstructure.lights;

import com.team841.calliope.constants.SC;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDIOSpark implements LEDIO{
    private final AddressableLED LED = new AddressableLED(5);
    private final AddressableLEDBuffer Buffer = new AddressableLEDBuffer(59);
    

    public LEDIOSpark() {
        LED.setLength(Buffer.getLength());
        LED.setData(Buffer);
        LED.start();
    }

    @Override
    public void updateInputs(LedIOInputs inputs) {
    //   inputs.sparkOutput = LED.get();
    }

    @Override
    public void setColor(String color){
        if(color == "red"){
            for(var i = 0; i < Buffer.getLength(); i++) 
            Buffer.setRGB(i, 255, 0, 0);
            LED.setData(Buffer);
        }
        if(color == "green"){
            for(var i = 0; i < Buffer.getLength(); i++) 
            Buffer.setRGB(i, 0, 255, 0);
            LED.setData(Buffer);
        }

    
    }

    @Override
    public void set(double value) {
    //    LED.set(value);
    }
}
