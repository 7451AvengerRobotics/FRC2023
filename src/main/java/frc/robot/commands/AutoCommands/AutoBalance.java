package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoBalance extends CommandBase {
    // Need to be tuned
    PIDController pid = new PIDController(0.024, 0.001, 0.002);
    double speed;
    double angle;
    Drivetrain drive;
    public static boolean stop;

    public AutoBalance(Drivetrain drive) {

        this.drive = drive;       
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        System.out.println("AUTO BALANCING ENABLED");
        stop = false;
    }

    @Override
    public void execute() {
        angle = drive.getGyroPitch();
        speed = 0.15;

        System.out.println("Balance " + speed);

        if (angle >  -4 && angle < 3) {
            drive.setPower(0);
        }
        else {
            drive.setPower(-0.15);
        }



    }

    @Override public void end(boolean interrupted) {
        System.out.println("BALANCE STOPPED");
        drive.setBreakMode();
        drive.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return stop;
    }
}
