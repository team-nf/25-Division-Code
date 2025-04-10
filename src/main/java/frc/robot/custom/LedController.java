// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.custom;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class LedController {

    private AddressableLED leds;
    private AddressableLEDBuffer buffer;

    private LEDPattern red = LEDPattern.solid(Color.kRed);

    public LedController(int channel)
    {
        leds = new AddressableLED(channel);

        buffer = new AddressableLEDBuffer(32);

        leds.setLength(buffer.getLength());

        leds.setData(buffer);

        leds.start();

    }

    public void setLedsRed()
    {
        red.applyTo(buffer);
        leds.setData(buffer);
    }
}
