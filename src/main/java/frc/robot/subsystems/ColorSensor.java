package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorSensor extends SubsystemBase {

    private final I2C.Port i2cPort;
    private ColorSensorV3 color;
    ColorMatchResult match;
    private final ColorMatch colorMatch;
    private final Color pTarget = new Color(255,0 , 255);
    private final Color yTarget = new Color(255, 255, 0);


    public ColorSensor(){
        super();

        this.i2cPort =  I2C.Port.kOnboard;
        color = new ColorSensorV3(i2cPort);
        this.colorMatch = new ColorMatch();
        colorMatch.addColorMatch(pTarget);
        colorMatch.addColorMatch(yTarget);
        

    }





    public int getProxmity(){
       return color.getProximity();
    }

    public double detectColor(){
        Color detectedColor = color.getColor();
        return detectedColor.green;
    }
    
    public boolean detectObject(){
        if(getProxmity() >120){
            return true;
        }else{
        return false;
        }
    }
    
}
