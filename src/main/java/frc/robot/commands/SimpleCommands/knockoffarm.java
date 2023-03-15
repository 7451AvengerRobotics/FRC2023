package frc.robot.commands.SimpleCommands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.armknockoff;

public class knockoffarm extends CommandBase{
    private final armknockoff arm;
    public knockoffarm(armknockoff arm){
        this.arm = arm;
        addRequirements(arm);
    }

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    @Override
    public void initialize(){

        led = new AddressableLED(9);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        ledBuffer = new AddressableLEDBuffer(60);
        led.setLength(ledBuffer.getLength());
    
        // Set the data
        led.setData(ledBuffer);
        led.start();

    }

    @Override 
    public void execute(){
    }
    
    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

