package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SimpleCommands.VFBAREncoder;
import frc.robot.commands.SimpleCommands.ArmCommands.ArmExtendCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.VirtualFourBar;



public class ComplexArmVfBar extends SequentialCommandGroup {
  public ComplexArmVfBar(Arm arm, VirtualFourBar bar, int encoderPos) {
    addCommands(
      new RunCommand(() -> arm.lockSolenoid()).andThen(      
            new ParallelDeadlineGroup(
                new WaitCommand(1.5),
                new ArmExtendCommand(arm),
                new VFBAREncoder(bar, arm, encoderPos)
      ))
   );
  }
}
