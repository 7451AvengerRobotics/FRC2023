

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


  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param left      The control input for driving lefts/backwards
   * @param right     The control input for turning
   * @param turbo     The button input for toggling the robot speed
   */
  public TankDrive(
      Drivetrain drive,
      DoubleSupplier left,
      DoubleSupplier right,
      BooleanSupplier turbo) {
        super();


    this.drive = drive;
    this.left = left;
    this.right = right;
    this.turbo = turbo;


    addRequirements(drive);
  }


  @Override
  public void execute() {
    if (turbo.getAsBoolean() == true){
    double scalar = turbo.getAsBoolean() ? 0.6 : 0.6;
    drive.tankDrive(right.getAsDouble() * scalar, left.getAsDouble() * scalar);
    SmartDashboard.putNumber("DriveTrain Power", left.getAsDouble());
    }else{
    double scalar = turbo.getAsBoolean() ? 0.5 : 0.5;
    drive.tankDrive(right.getAsDouble() * scalar, left.getAsDouble() * scalar);
    SmartDashboard.putNumber("DriveTrain Power", left.getAsDouble());
    }
  }
}


