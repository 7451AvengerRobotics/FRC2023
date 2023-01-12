// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//More Imports
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

//Creating Intake Class with SubsystemBase as an extension
public class Pneumatics extends SubsystemBase {
  private static boolean isExtended;

  // Creating Compressor and Solenoid Classes
  private final Compressor compressor;
  private final DoubleSolenoid intakeSolenoid;

  // Creating Intake Motors

  /**
   * creates a new intake class.
   */
  public Pneumatics() {
    super();

    // init subsystem class

    compressor = new Compressor(39, PneumaticsModuleType.REVPH);
    intakeSolenoid = new DoubleSolenoid(
    39,
    PneumaticsModuleType.REVPH,
    PortConstants.INTAKE_PNEUMATICS_PORTS[0],
    PortConstants.INTAKE_PNEUMATICS_PORTS[1]
    );
  }

  // Method Stoping Pneumatics System
  public void stop() {
    compressor.disable();
  }
  
  // Method Extending Intake with Solenoids & Pneumatic System
  public void extend() {
    intakeSolenoid.set(Value.kForward);
    isExtended = true;
  }

  // Method Retracting Intake with Solenoids & Pneumatic System
  public void retract() {
    intakeSolenoid.set(Value.kReverse);
    isExtended = false;
  }

  /**
   * If extended, retract.
   */
  public void toggle() {
    if (isExtended) {
      retract();
    } else {
      extend();
    }
  }

  /**
   * set the power of the intake.
   */
  public void power(double speed) {
    }
  }