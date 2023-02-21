package frc.robot.commands.SimpleCommands.VirtualFourBarCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VirtualFourBar;

public class VirtualFourBarCommand extends CommandBase{
    private final VirtualFourBar bar;
    private final double encoderPos;
    public VirtualFourBarCommand(VirtualFourBar bar, double encoderPos){
        super();
        this.bar = bar;
        this.encoderPos = encoderPos;
        addRequirements(bar);
    }

    @Override
    public void initialize(){}

    @Override 
    public void execute(){
        bar.setPosition(encoderPos);
    }
    
    @Override
    public void end(boolean interrupted){
        bar.setPower(0);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

