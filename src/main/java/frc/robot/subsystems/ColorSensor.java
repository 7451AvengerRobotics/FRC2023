package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorSensor extends SubsystemBase {

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 color;
    private final Color BlueColorTarget; 
    private final Color RedColorTarget; 
    private final ColorMatch colorMatcher;
    private static ColorMatchResult ClosestColor;
    private static Color detectedColor;


    public ColorSensor(){
        super();

        color = new ColorSensorV3(i2cPort);
        BlueColorTarget = new Color(0,0,255); //needs to change depending on the rgb value of the blue ball
        RedColorTarget = new Color(255,0,0); //needs to change depending on the rgb value of the red ball
        colorMatcher = new ColorMatch();

        colorMatcher.addColorMatch(BlueColorTarget);
        colorMatcher.addColorMatch(RedColorTarget);



    }

    public boolean checkifBlue(){
      detectedColor = color.getColor();  
      ClosestColor = colorMatcher.matchClosestColor(detectedColor);

        if(ClosestColor.color == BlueColorTarget){
            return true;
        }
            return false;
    }

    public boolean checkifRed(){
      detectedColor = color.getColor();  
      ClosestColor = colorMatcher.matchClosestColor(detectedColor);

      if(ClosestColor.color == RedColorTarget){
          return true;
      }
      return false;
    }
}
