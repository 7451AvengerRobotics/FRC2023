package frc.robot.commands.SimpleCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class StandardTurret extends CommandBase{
    private final Turret turret;
    private final int power;
    public StandardTurret(Turret turret, int power){
        this.power = power;
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize(){
    }

    @Override 
    public void execute(){

        turret.setPosition(power);
    }
    
    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){        
        return false;
    }
}

