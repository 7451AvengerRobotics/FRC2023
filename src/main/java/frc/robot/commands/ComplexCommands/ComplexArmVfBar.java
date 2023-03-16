package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.SimpleCommands.VFBAREncoder;
import frc.robot.commands.SimpleCommands.ArmCommands.ArmExtendCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.VirtualFourBar;



public class ComplexArmVfBar extends ParallelCommandGroup {
  public ComplexArmVfBar(Arm arm, VirtualFourBar bar, int encoderPos) {
    if(arm.getArmState() == false){
      new ArmExtendCommand(arm);
    }
    addCommands(   
                //extends the Arm 
              
                // Sets the virtual 4 bar to a desired location.
                new VFBAREncoder(bar, arm, encoderPos)
   );
  }
}
