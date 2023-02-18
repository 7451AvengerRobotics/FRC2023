package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class Claw extends SubsystemBase{
 

    private final Compressor compressor;
    private final DoubleSolenoid clawSolenoid;
    private final CANSparkMax clawMotor;
    private static boolean isExtended;


    public Claw(){
        super();

        compressor = new Compressor(1, PneumaticsModuleType.REVPH); //need to change module type
        clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PortConstants.CLAW_PNEUMATIC[0], PortConstants.CLAW_PNEUMATIC[1]);

        clawMotor = new CANSparkMax(PortConstants.Claw, MotorType.kBrushless);
    }


    public void stop(){
        compressor.disable();
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
        clawMotor.set(power);
    }


}
