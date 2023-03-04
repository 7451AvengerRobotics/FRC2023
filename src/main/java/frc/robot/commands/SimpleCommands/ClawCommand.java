package frc.robot.commands.SimpleCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawCommand extends CommandBase{
    private final Claw claw;
    private final double power;
    public ClawCommand(Claw claw, double power){
        this.claw = claw;
        this.power = power;
        addRequirements(claw);
    }

    @Override
    public void initialize(){
    }

    @Override 
    public void execute(){
        claw.setPower(power);
    }
    
    @Override
    public void end(boolean interrupted){
        claw.setPower(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

