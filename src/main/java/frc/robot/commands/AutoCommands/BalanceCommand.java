package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AutoBalance;
import frc.robot.subsystems.Drivetrain;

public class BalanceCommand extends CommandBase {
    private final Drivetrain drive;
    public BalanceCommand(Drivetrain drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        AutoBalance.run();
    }

    @Override public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
