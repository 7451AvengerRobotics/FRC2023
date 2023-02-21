package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SimpleCommands.ArmCommands.ArmExtendCommand;
import frc.robot.commands.SimpleCommands.ClawCommands.ClawTestCommand;
import frc.robot.commands.SimpleCommands.VirtualFourBarCommands.VirtualFourBarCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.VirtualFourBar;

public class mediumGoalCommand extends SequentialCommandGroup {
    public mediumGoalCommand(Turret turret, VirtualFourBar virtualFourBar, Arm arm, Claw claw){
        addCommands(
            new ArmExtendCommand(arm),
            new VirtualFourBarCommand(virtualFourBar, -2000),
            new ClawTestCommand(claw, -0.4)
        );
    }
    
}
