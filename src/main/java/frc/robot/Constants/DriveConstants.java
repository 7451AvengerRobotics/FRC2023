package frc.robot.Constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DriveConstants {

  public static final int INPUT_ROLLING_AVERAGE_SAMPLE_SIZE = 20;
  
  // Voltage
  public static final double KS_VOLTS = 0.051905;
  public static final double KV_VOLT_SECONDS_PER_METER = 2.9014;
  public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.39614;

  public static final double KP_DRIVE_VELOCITY = 0.9638;

  // Differential Drive Kinematics
  public static final double K_TRACK_WIDTH_METERS = 0.5842;


  public final static DifferentialDriveKinematics K_DRIVE_KINEMATICS = new DifferentialDriveKinematics  (DriveConstants.K_TRACK_WIDTH_METERS);

  // Max Velocity/Acceleration
  public static final double K_MAX_SPEED_METER_PER_SECOND = 3;
  public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;

  // RAMSETE Parameters
  public static final double K_RAMSETE = 2;
  public static final double K_RAMSETE_ZETA = 0.7;
  public static final double MAX_DRIVE_VOLTAGE = 5;




  //Shooter Constants; Did not wanna create a new class
    /**
   * @param velocityCounts Falcon Velocity Counts
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return RPM of Mechanism
   */
  public static double falconToRPM(double velocityCounts, double gearRatio) {
    double motorRPM = velocityCounts * (600.0 / 2048.0);
    double mechRPM = motorRPM / gearRatio;
    return mechRPM;
  }

  /**
   * @param RPM RPM of mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return RPM of Mechanism
   */
  public static double RPMToFalcon(double RPM, double gearRatio) {
    double motorRPM = RPM * gearRatio;
    double sensorCounts = motorRPM * (2048.0 / 600.0);
    return sensorCounts;
  }


  public static final double GEAR_RATIO = 12.75;
  public static final double wheelCircumferenceInches = 8 * Math.PI;
  public static final double ENCODER_EDGES_PER_REV = 2048;

  public static final double conversionForFalconUnits = (ENCODER_EDGES_PER_REV * Units.inchesToMeters(wheelCircumferenceInches));

}
