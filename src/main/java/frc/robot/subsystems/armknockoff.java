package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class armknockoff extends SubsystemBase{
    
    private final WPI_TalonFX miniarm;
    private final Solenoid ratchet;
    private final DoubleSolenoid HUGEPISTON;
    private final Compressor comp;

    public armknockoff(){
        miniarm = new WPI_TalonFX(PortConstants.VirtualFourBar);
        ratchet = new Solenoid(PneumaticsModuleType.CTREPCM, PortConstants.Arm[2]);
        HUGEPISTON = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PortConstants.Arm[0], PortConstants.Arm[1]);
        comp = new Compressor(0, PneumaticsModuleType.CTREPCM);
        comp.enableDigital();
    }

    public void stop(){
        comp.disable();
    }

    public void extret(){
        HUGEPISTON.toggle();
    }

    public void rUnlock(){
        ratchet.set(true);
    }

    public void rLock(){
        ratchet.set(false);
    }

    public void movebar(double power){
        miniarm.set(ControlMode.PercentOutput, power);
    }
}
