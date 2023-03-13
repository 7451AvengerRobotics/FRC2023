package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveBackAuto extends CommandBase{
    private final Drivetrain drive;
    private final double power;
    public DriveBackAuto(Drivetrain drive, double power){
        this.drive = drive;
        this.power = power;
        addRequirements(drive);
    }

    @Override
    public void initialize(){
    }

    @Override 
    public void execute(){
        drive.setPower(power);
    }
    
    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

