package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SimpleCommands.SolenoidExtendCommand;
import frc.robot.commands.SimpleCommands.SolenoidRetractCommand;
import frc.robot.commands.SimpleCommands.VirtualFourBarCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.VirtualFourBar;

public class VirtualFourBarComplex extends SequentialCommandGroup {
    VirtualFourBar miniArm;
    Arm arm;
    double power;
    public VirtualFourBarComplex(VirtualFourBar miniArm, Arm arm, double power){
        addCommands(
            new SolenoidRetractCommand(arm), 
            new VirtualFourBarCommand(miniArm, arm, power),
            new SolenoidExtendCommand(arm)
        );

    }

    
}
