package frc.robot.commands.DriveTypes;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TankDrive extends CommandBase {
  private final Drivetrain drive;
  private final DoubleSupplier left;
  private final DoubleSupplier right;
  private final BooleanSupplier turbo;
  private final BooleanSupplier precision;

  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param left      The control input for driving lefts/backwards
   * @param right     The control input for turning
   * @param turbo     The button input for toggling the robot speed
   * @param precision The button input for slowing robot speed
   */
  public TankDrive(
      Drivetrain subsystem,
      DoubleSupplier left,
      DoubleSupplier right,
      BooleanSupplier turbo,
      BooleanSupplier precision) {
        super();

    this.drive = subsystem;
    this.left = left;
    this.right = right;
    this.turbo = turbo;
    this.precision = precision;

    addRequirements(drive);
  }

  @Override
  public void execute() {
    // Assign Base Scalar
    double scalar = turbo.getAsBoolean() ? 0.7 : 0.7;

    // Assign Scalar Based on `turbo` and `precision`
    if (turbo.getAsBoolean()) {
      scalar = 1.0;
    } else if (precision.getAsBoolean()) {
      scalar = turbo.getAsBoolean() ? 0.4 : 0.4;
    }

    // Drive Robot and Report Values
    drive.tankDrive(left.getAsDouble() * -scalar, right.getAsDouble() * -scalar);
    SmartDashboard.putNumber("DriveTrain Power", left.getAsDouble());
  }
}