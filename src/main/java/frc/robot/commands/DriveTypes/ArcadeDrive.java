package frc.robot.commands.DriveTypes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ArcadeDrive extends CommandBase {
  private final Drivetrain drive;
  private final Arm arm;
  private final DoubleSupplier power;
  private final DoubleSupplier turn;
  private final BooleanSupplier turbo;
  /**
   * Creates a new DefaultDrive.
   *
   * @param drive       The drive subsystem this command wil run on.
   * @param Arm         The arm subsystem is required to gather the state of the arm. If extended, then the robot will move slower
   * @param power       The control input for driving 
   * @param rotation    The control input for turning
   * @param turbo       The button input for toggling the robot speed
   */

   
  public ArcadeDrive( Drivetrain drive, Arm arm, DoubleSupplier power, 
                    DoubleSupplier turn, BooleanSupplier turbo) {
        super();

    this.drive = drive;
    this.arm = arm;
    this.power = power;
    this.turn = turn;
    this.turbo = turbo;
    addRequirements(drive, arm);
  }

  @Override
  public void execute() {
    if (arm.getArmState() == true){
      double scalar = arm.getArmState() ? 0.5: 0.5;
      drive.arcadeDrive(power.getAsDouble() * scalar, turn.getAsDouble() * -scalar);
    }
    else if(turbo.getAsBoolean() == true){
      double scalar = turbo.getAsBoolean() ? 1: 1;
      drive.arcadeDrive(power.getAsDouble() * scalar, turn.getAsDouble() * -scalar);
    }
    else{
      double scalar = turbo.getAsBoolean() ? 0.7 : 0.7;
      drive.arcadeDrive(power.getAsDouble() * scalar, turn.getAsDouble() * -scalar);
    }



  }
 }