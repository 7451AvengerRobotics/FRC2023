package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase{
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private int m_rainbowFirstPixelHue;

    public Led(int port, int length){
        // PWM port 
        // Must be a PWM header, not MXP or DIO
        led = new AddressableLED(port);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        ledBuffer = new AddressableLEDBuffer(length);
        led.setLength(ledBuffer.getLength());

        // Set the data

        led.setData(ledBuffer);
        rainbow();
        led.setData(ledBuffer);
        led.start();
        m_rainbowFirstPixelHue = 0;
    }

    public void setColor(int R, int G, int B){
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            ledBuffer.setRGB(i, R, G, B);
         }
         
         led.setData(ledBuffer);
    }


    public void rainbow() {
        // For every pixel
        
        for (var i = 0; i < ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
          // Set the value
          ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
      }
}
