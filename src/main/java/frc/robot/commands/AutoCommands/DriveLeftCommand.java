package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveLeftCommand extends CommandBase{
    private final Drivetrain drive;
    private final double power;
    public DriveLeftCommand(Drivetrain drive, double power){
        this.drive = drive;
        this.power = power;
        addRequirements(drive);
    }

    @Override
    public void initialize(){
    }

    @Override 
    public void execute(){
        drive.setPowerLeft(power);
    }
    
    @Override
    public void end(boolean interrupted){
        drive.setBreakMode();
        drive.tankDrive(0, 0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

