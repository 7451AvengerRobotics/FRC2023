package frc.robot.commands.AutoCommands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.VirtualFourBar;



public class Encoder5Auto extends SequentialCommandGroup {
  public Encoder5Auto(Arm arm, Drivetrain drive, double drivepower, VirtualFourBar bar, Claw claw, double clawpower, Turret turret, Pigeon2 gyro) {
    addCommands(
      Commands.sequence(
        //new TurretTestCommand(turret, 0.5).withTimeout(1.30),
        //new ArmExtendCommand(arm).withTimeout(2),
        //new ClawOuttake(claw, clawpower).withTimeout(1),
        //new ArmRetractCommand(arm).withTimeout(3),
        //new TurretTestCommand(turret, -0.5).withTimeout(1.4),
        new DriveBackAuto(drive, drivepower).withTimeout(1.5)),
        new GetOnRamp(drive).withTimeout(3),
        new BalanceCommand(drive).withTimeout(5),
        new DriveBackAuto(drive, -drivepower).withTimeout(0.05)

);
  }
}
