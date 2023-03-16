package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AutoBalance;

public class BalanceCommand extends CommandBase {
    public BalanceCommand() {
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
