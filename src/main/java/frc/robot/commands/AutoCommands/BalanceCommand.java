package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class BalanceCommand extends CommandBase {
    // Need to be tuned
    PIDController pid = new PIDController(0.024, 0.001, 0.002);
    double speed;
    double angle;
    Drivetrain drive;
    public static boolean stop;

    public BalanceCommand(Drivetrain drive) {

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
        speed = pid.calculate(angle, 0);

        System.out.println("Balance " + speed);

        if (speed < -0.2) {
            drive.setPower(-0.135);
        } else if (speed > 0.2) {
            drive.setPower(0.135);
        } else {
            drive.setPower(speed);
        }

        if (angle > (2 - 15) && angle < (2 + 10)) {
            System.out.println("ANGLE REACHED");
            
            stop = true;
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
