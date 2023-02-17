package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretTestCommand extends CommandBase{
    private final Turret turret;
    private final double power;
    public TurretTestCommand(Turret turret, double power){
        this.turret = turret;
        this.power = power;
    }

    @Override 
    public void execute(){
        turret.turn(power);
    }
    
    @Override
    public void end(boolean interrupted){
        turret.turn(0);
    }
}