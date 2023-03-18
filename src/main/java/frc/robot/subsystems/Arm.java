package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Constants.PortConstants;

public class Arm extends SubsystemBase{
    private static boolean isExtended = false;
    private final Compressor compressor;
    private final DoubleSolenoid armSolenoid;
    private final Solenoid lockSolenoid;

    public Arm() {
        super();
        //initializing compressor and solenoid  
       compressor = new Compressor(0, PneumaticsModuleType.CTREPCM); //need to change module id 
        armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PortConstants.Arm[0],PortConstants.Arm[2]); 
        lockSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 5);
        //need to change module id
    }
   public void stop(){
    compressor.disable();
   }  

   public void start(){
        compressor.enableDigital();
    }  

   public void extend(){
    //sets the solenoid output to on
    armSolenoid.set(Value.kReverse);
    isExtended = true;
   }

   public void retract(){
    //sets the solenoid output to off
    armSolenoid.set(Value.kForward);
    isExtended = false;
   }

   public void toggle(){
    if(isExtended){
        this.retract();
    } else{
        this.extend();
    }
   }


   public void unlockSolenoid(){
    lockSolenoid.set(false);
   }

   public void lockSolenoid(){
    lockSolenoid.set(true);
   }

   public boolean getArmState(){
    return isExtended;
   }

}

