package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SimpleCommands.TurretTestCommand;
import frc.robot.commands.SimpleCommands.ClawCommands.ClawExtend;
import frc.robot.commands.SimpleCommands.ClawCommands.ClawIntake;
import frc.robot.commands.SimpleCommands.VirtualFourBar.EncoderandArm;
import frc.robot.commands.SimpleCommands.VirtualFourBar.ResetVFbarEncoder;
import frc.robot.commands.SimpleCommands.VirtualFourBar.StandardEncoder;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.VirtualFourBar;



public class TwoElementAuto extends SequentialCommandGroup {
  public TwoElementAuto(Arm arm, Drivetrain drive, double drivepower, Claw claw, double clawpower, Turret turret, VirtualFourBar bar) {
    addCommands(
      Commands.sequence( 
        new TurretTestCommand(turret, 0.5).withTimeout(1.30),
        new EncoderandArm(bar, arm, 40000).withTimeout(2.5),
        new ClawExtend(claw).withTimeout(1),
        new ResetVFbarEncoder(bar, arm, 0).withTimeout(0.3),
        new TurretTestCommand(turret, -0.5).withTimeout(1.30),
        new DriveBackAuto(drive, drivepower).withTimeout(1),
        new DriveLeftCommand(drive, 0.20).withTimeout(0.13), 
        new ParallelCommandGroup(
        new StandardEncoder(bar, arm, 76000).withTimeout(0.5),
        new ClawIntake(claw, 1).withTimeout(4), 
        new DriveBackAuto(drive, drivepower).withTimeout(2.89)),
        new ResetVFbarEncoder(bar, arm, 0)
        ));
  }
}
