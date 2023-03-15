package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SimpleCommands.SolenoidExtendCommand;
import frc.robot.commands.SimpleCommands.SolenoidRetractCommand;
import frc.robot.commands.SimpleCommands.VirtualFourBarCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.VirtualFourBar;

public class VirtualFourBarComplex extends SequentialCommandGroup {
    VirtualFourBar miniArm;
    Arm arm;
    double power;
    Led led;
    public VirtualFourBarComplex(VirtualFourBar miniArm, Arm arm, Led led, double power){
        addCommands(
            new SolenoidExtendCommand(arm),
            new VirtualFourBarCommand(miniArm, arm, power),
            new SolenoidRetractCommand(arm),
            new RunCommand(() -> led.setColor(0,0,0))
        );

    }

    
}
