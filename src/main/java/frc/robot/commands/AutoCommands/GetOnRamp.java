package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class GetOnRamp extends CommandBase{
    
    boolean end;
    float startTime, timePassed;
    Drivetrain drive;
    public GetOnRamp(Drivetrain drive) {
        this.drive = drive;
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
        if(pitch != 15.8) {
            drive.setPower(0.525);
        }

        if (pitch < 18 && pitch > 14) {
            end = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.setBreakMode();
        drive.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return end;
    }

}
