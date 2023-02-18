package frc.robot.commands.SimpleCommands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
public class ClawRetractCommand extends CommandBase{

    private final Claw claw;


    public ClawRetractCommand(Claw claw){
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        claw.retract();
    }

    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return true;
    }
}
