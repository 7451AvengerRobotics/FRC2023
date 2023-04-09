package frc.robot.commands.SimpleCommands.VirtualFourBar;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.VirtualFourBar;

public class ResetVFbarEncoder extends CommandBase{
    private final VirtualFourBar bar;
    private final Arm arm;
    private final double power;
    public ResetVFbarEncoder(VirtualFourBar bar, Arm arm, double power){
        this.bar = bar;
        this.power = power;
        this.arm = arm;
        addRequirements(bar);
    }

    @Override
    public void initialize(){
        arm.lockSolenoid();
    }

    @Override 
    public void execute(){
        arm.extend();
        bar.setPosition(power);
    }
    
    @Override
    public void end(boolean interrupted){
        arm.lockSolenoid();
    }

    @Override
    public boolean isFinished(){        
        return false;
    }
}

