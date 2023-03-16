package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.AutoBalance;
import frc.robot.subsystems.Drivetrain;

public class BalanceCommand extends CommandBase {
    // Need to be tuned
    PIDController pid = new PIDController(DriveConstants.KP_DRIVE_VELOCITY, 0, 0);
    double speed;
    double angle;
    double speeddBalance;
    Drivetrain drive;
    double speedBalance;
    public static boolean stop;

    public BalanceCommand(double speedBalance) {
        addRequirements(drive);
        this.speedBalance = speeddBalance;
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

        if (speed > -0.5) {
            drive.tankDrive(speedBalance, speedBalance);
        } else if (speed > 0.5) {
            drive.tankDrive(-(speedBalance), -(speedBalance));
        } else {
            drive.tankDrive(-speed, -speed);
        }

        if (angle > (0 - 3) && angle < (0 + 3)) {
            System.out.println("ANGLE REACHED");
            stop = true;
        }

    }

    @Override public void end(boolean interrupted) {
        System.out.println("BALANCE STOPPED");
        drive.setBreakMode();
    }

    @Override
    public boolean isFinished() {
        return stop;
    }
}
