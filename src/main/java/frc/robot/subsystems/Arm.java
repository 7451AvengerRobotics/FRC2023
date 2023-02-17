package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Constants.PortConstants;

public class Arm {
    private static boolean isExtended;
    private final Compressor compressor;
    private final DoubleSolenoid armSolenoid;

    public Arm() {
        super();
        //initializing compressor and solenoid  
        compressor = new Compressor(1, PneumaticsModuleType.REVPH); //need to change module id 
        armSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PortConstants.Arm[0],PortConstants.Arm[1] ); //need to change module id
    }
   public void stop(){
    compressor.disable();
   }  

   public void extend(){
    //sets the solenoid output to on
    armSolenoid.set(Value.kForward);
    isExtended = true;
   }

   public void retract(){
    //sets the solenoid output to off
    armSolenoid.set(Value.kReverse);
    isExtended = false;
   }

   public void toggle(){
    if(isExtended){
        this.retract();
    } else{
        this.extend();
    }
   }

}

