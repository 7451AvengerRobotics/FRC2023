package frc.robot.commands.SimpleCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VirtualFourBar;

public class VirtualFourBarCommand extends CommandBase{
    private final VirtualFourBar bar;
    private final double power;
    public VirtualFourBarCommand(VirtualFourBar bar, double power){
        this.bar = bar;
        this.power = power;
        addRequirements(bar);
    }

    @Override
    public void initialize(){
    }

    @Override 
    public void execute(){
        bar.setPower(power);
    }
    
    @Override
    public void end(boolean interrupted){
        bar.setPower(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

