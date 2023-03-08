package frc.robot.commands.DriveTypes;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ArcadeDrive extends CommandBase {
  private final Drivetrain drive;
  private final DoubleSupplier forward;
  private final DoubleSupplier turn;
  private final BooleanSupplier turbo;
  private final SlewRateLimiter slewRate;
  private final BooleanSupplier slow;

  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param left      The control input for driving backwards
   * @param right     The control input for turning
   * @param turbo     The button input for toggling the robot speed
   */
  public ArcadeDrive(
      Drivetrain subsystem, DoubleSupplier left, DoubleSupplier right, BooleanSupplier turbo, BooleanSupplier slow) {
        super();

    this.drive = subsystem;
    this.forward = left;
    this.turn = right;
    this.turbo = turbo;
    this.slow = slow;
    slewRate = new SlewRateLimiter(12);

    addRequirements(drive);
  }

  @Override
  public void execute() {
    if (slow.getAsBoolean() == true){
      double scalar = slow.getAsBoolean() ? 0.3: 0.3;
      drive.arcadeDrive(slewRate.calculate(forward.getAsDouble() * scalar), turn.getAsDouble() * -scalar);
    }
    else if(turbo.getAsBoolean() == true){
      double scalar = turbo.getAsBoolean() ? 1: 1;
      drive.arcadeDrive(slewRate.calculate(forward.getAsDouble() * scalar), turn.getAsDouble() * -scalar);
    }
    else{
      double scalar = turbo.getAsBoolean() ? .5 : .5;
      drive.arcadeDrive(slewRate.calculate(forward.getAsDouble() * scalar), turn.getAsDouble() * -scalar);
    }
  }
 }