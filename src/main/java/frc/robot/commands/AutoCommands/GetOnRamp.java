package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class GetOnRamp extends CommandBase{
    
    boolean end;
    float startTime, timePassed;
    Drivetrain drive;
    public GetOnRamp(Drivetrain drive) {
        addRequirements(drive);
        end = false;
    }

    @Override
    public void initialize() {
        System.out.println("started");
    }

    @Override
    public void execute() {
        System.out.println("RAMP");
        double pitch = Math.abs(drive.getGyroPitch());
        if(pitch != 17) {
            drive.tankDrive(-0.525, -0.525);
        }

        if (pitch < 35 && pitch > 33) {
            end = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.setBreakMode();
        drive.tankDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return end;
    }

}
