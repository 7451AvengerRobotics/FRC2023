package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SimpleCommands.ClawCommands.ClawOuttake;
import frc.robot.commands.SimpleCommands.ArmCommands.ArmExtendCommand;
import frc.robot.commands.SimpleCommands.ArmCommands.ArmRetractCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;



public class ComplexAuto extends SequentialCommandGroup {
  public ComplexAuto(Arm arm, Drivetrain drive, double drivepower, Claw claw, double clawpower) {
    addCommands(
      Commands.deadline(
      new WaitCommand(1), 
      new ArmExtendCommand(arm)),
      new WaitCommand(4),
      new ClawOuttake(claw, clawpower),
      new WaitCommand(3),
      new ArmRetractCommand(arm),
      new WaitCommand(1),
      new DriveBackAuto(drive, drivepower)
    );
  }
}
