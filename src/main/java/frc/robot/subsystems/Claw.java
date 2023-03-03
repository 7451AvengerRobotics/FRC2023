package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class Claw extends SubsystemBase{
 

   
    private final DoubleSolenoid clawSolenoid;
    private final CANSparkMax clawMotorL;
    private final CANSparkMax clawMotorR;
    private static boolean isExtended;


    public Claw(){
        super();

        
        clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PortConstants.CLAW_PNEUMATIC[0], PortConstants.CLAW_PNEUMATIC[1]);
        clawMotorL = new CANSparkMax(PortConstants.Claw[0], MotorType.kBrushless);
        clawMotorR = new CANSparkMax(PortConstants.Claw[1], MotorType.kBrushless);
    }


   

    public void extend(){
        clawSolenoid.set(Value.kForward);
        isExtended = true;
    }

    public void retract(){
        clawSolenoid.set(Value.kReverse);
        isExtended = false;
    }

    public void toggle(){
        if(isExtended){
            this.retract();
        }else{
            this.extend();
        }
    }

    public void setPower(double power){
        clawMotorL.set(power);
        clawMotorR.set(power);
    }


}
