package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SimpleCommands.TurretTestCommand;
import frc.robot.commands.SimpleCommands.ClawCommands.ClawExtend;
import frc.robot.commands.SimpleCommands.VirtualFourBar.EncoderandArm;
import frc.robot.commands.SimpleCommands.VirtualFourBar.ResetVFbarEncoder;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.VirtualFourBar;



public class EncoderAuto extends SequentialCommandGroup {
  public EncoderAuto(Arm arm, Drivetrain drive, double drivepower, Claw claw, double clawpower, Turret turret, VirtualFourBar bar) {
    addCommands(
      Commands.sequence( 
        new TurretTestCommand(turret, 0.5).withTimeout(1.35),
        new EncoderandArm(bar, arm, 38500).withTimeout(1),
        new ClawExtend(claw).withTimeout(1),
        new ResetVFbarEncoder(bar, arm, 0).withTimeout(0.3),
        new TurretTestCommand(turret, -0.5).withTimeout(1.35),
        new DriveBackAuto(drive, drivepower).withTimeout(4))
        );
  }
}
