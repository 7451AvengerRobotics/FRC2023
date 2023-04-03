package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class Claw extends SubsystemBase{
 

//Creating Claw and properties of it 
    private final Solenoid clawSolenoid;
    private final CANSparkMax clawMotorL;
    private final CANSparkMax clawMotorR;
    private static boolean isExtended;


    public Claw(){
        super();

 //Creating Claw and properties of it        
        clawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, PortConstants.CLAW_PNEUMATIC[0]);
        clawMotorL = new CANSparkMax(PortConstants.Claw[0], MotorType.kBrushless);
        clawMotorR = new CANSparkMax(PortConstants.Claw[1], MotorType.kBrushless);
        
    }


   
//Extending the Claw
    public void extend(){
        clawSolenoid.set(true);
        isExtended = true;
    }

//Retracting the Claw
    public void retract(){
        clawSolenoid.set(false);
        isExtended = false;
    }
//Toggling the Claw
    public void toggle(){
        if(isExtended){
            this.retract();
        }else{
            this.extend();
        }
    }
//Setting power to the claw
    public void setPower(double power){
        clawMotorL.set(power);
        clawMotorR.set(-power);
    }


}
