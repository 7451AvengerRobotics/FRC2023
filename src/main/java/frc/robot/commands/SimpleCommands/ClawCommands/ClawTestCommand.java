package frc.robot.commands.SimpleCommands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawTestCommand extends CommandBase{
    private final Claw claw;
    private final double power;
    public ClawTestCommand(Claw claw, double power){
        super();
        this.claw = claw;
        this.power = power;
        addRequirements(claw);
    }

    @Override
    public void initialize(){}

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
        return true;
    }
}

