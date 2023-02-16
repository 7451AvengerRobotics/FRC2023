package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PortConstants;

//Creates index class 
public class SingleMotor extends SubsystemBase {
  // Creates the index motor
  // private final WPI_VictorSPX indexMotor;
  private final WPI_TalonFX motor;

  /**
   * creates a new index subsystem.
   */
  public SingleMotor() {
    super();

    motor = new WPI_TalonFX(PortConstants.TestMotor);
  }

  /**
   * set the index to a certain power.
   */
  public void power(double speed) {
    motor.set(speed);
  }
}