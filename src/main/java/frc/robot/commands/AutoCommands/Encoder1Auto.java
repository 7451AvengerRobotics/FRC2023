package frc.robot.commands.AutoCommands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SimpleCommands.TurretTestCommand;
import frc.robot.commands.SimpleCommands.ArmCommands.ArmExtendCommand;
import frc.robot.commands.SimpleCommands.ArmCommands.ArmRetractCommand;
import frc.robot.commands.SimpleCommands.ClawCommands.ClawOuttake;
import frc.robot.commands.SimpleCommands.VirtualFourBar.VirtualFourBarCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.VirtualFourBar;



public class Encoder1Auto extends SequentialCommandGroup {
  public Encoder1Auto(Arm arm, Drivetrain drive, double drivepower, VirtualFourBar bar, Claw claw, double clawpower, Turret turret, Pigeon2 gyro) {
    addCommands(
      Commands.sequence(
        new TurretTestCommand(turret, 0.5).withTimeout(1.25),
        new ArmExtendCommand(arm).withTimeout(2),
        new VirtualFourBarCommand(bar, arm, 0.3).withTimeout(0.55),
        new ClawOuttake(claw, clawpower).withTimeout(0.5),
        new VirtualFourBarCommand(bar, arm, -0.3).withTimeout(0.55),
        new ArmRetractCommand(arm).withTimeout(3),
        new TurretTestCommand(turret, -0.5).withTimeout(1.35),
        new DriveBackAuto(drive, drivepower).withTimeout(1.5)),
        new GetOnRamp(drive).withTimeout(3),
        new BalanceCommand(drive).withTimeout(3)
);
  }
}
