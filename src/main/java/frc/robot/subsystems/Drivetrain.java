package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  /* Calling the motors into arrays. Contains 4 falcons */
  protected final WPI_TalonFX[] leftMotors;
  protected final WPI_TalonFX[] rightMotors;
  private final DifferentialDrive differentialDrive;
  private final Pigeon2 gyro;
  private final DifferentialDriveOdometry odometry;
  static boolean autobalanceXMode;
  static final double kOffBalanceAngleThresholdDegrees = 10;
  static final double kOnBalanceAngleThresholdDegrees = 5;
  Pose2d pose;
  public Field2d m_field = new Field2d();

  /*
   * Calling the different items needed for the autonomous and differential drive
   */
  DifferentialDriveKinematics kinematics;
  SimpleMotorFeedforward feedForward;
  PIDController leftPIDController;
  PIDController rightPIDController;

  /**
   * this method is called when the DriveTrainSubsystem class is initialized.
   */
  public Drivetrain() {
    super();

    /* Instantiating the Motors */
    this.leftMotors = new WPI_TalonFX[] {
        new WPI_TalonFX(PortConstants.LEFT_DRIVE[0]),
        new WPI_TalonFX(PortConstants.LEFT_DRIVE[1])
    };
    this.rightMotors = new WPI_TalonFX[] {
        new WPI_TalonFX(PortConstants.RIGHT_DRIVE[0]),
        new WPI_TalonFX(PortConstants.RIGHT_DRIVE[1])
    };

    // Creating the differential drive and items needed for autonomous
    differentialDrive = new DifferentialDrive(leftMotors[0], rightMotors[0]);
    gyro = new Pigeon2(PortConstants.Gyro);
    odometry = new DifferentialDriveOdometry(
        this.getHeading(),
        encoderTicksToMeters(leftMotors[0].getSelectedSensorPosition()),
        encoderTicksToMeters(rightMotors[0].getSelectedSensorPosition()));

    // Instantiating the Autonomous and Kinematics
    kinematics = new DifferentialDriveKinematics(DriveConstants.K_TRACK_WIDTH_METERS);
    feedForward = new SimpleMotorFeedforward(DriveConstants.KS_VOLTS,
        DriveConstants.KV_VOLT_SECONDS_PER_METER,
        DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER);


    //Configuring the motors to run based of encoders when we need them for path planner
    leftMotors[0].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftMotors[1].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightMotors[0].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightMotors[1].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // Settings for motors to ensure they run properly
    leftMotors[1].follow(leftMotors[0]);
    rightMotors[1].follow(rightMotors[0]);

    rightMotors[0].setInverted(true);
    rightMotors[1].setInverted(InvertType.FollowMaster);

    // leftMotors[0].setNeutralMode(NeutralMode.Coast);
    // leftMotors[1].setNeutralMode(NeutralMode.Coast);
    // rightMotors[0].setNeutralMode(NeutralMode.Coast);
    // rightMotors[1].setNeutralMode(NeutralMode.Coast);

    // leftMotors[0].configOpenloopRamp(1);
    // leftMotors[1].configOpenloopRamp(1);
    // rightMotors[0].configOpenloopRamp(1);
    // rightMotors[1].configOpenloopRamp(1);

    // leftMotors[0].configFactoryDefault();
    leftMotors[1].setNeutralMode(NeutralMode.Brake);
    leftMotors[0].setNeutralMode(NeutralMode.Brake);
    rightMotors[0].setNeutralMode(NeutralMode.Brake);
    rightMotors[1].setNeutralMode(NeutralMode.Brake);

    leftMotors[0].configOpenloopRamp(0.7);
    leftMotors[1].configOpenloopRamp(0.7);
    rightMotors[0].configOpenloopRamp(0.7);
    rightMotors[1].configOpenloopRamp(0.7);

    resetEncoders();
    setBreakMode();
    resetGyro();


    Shuffleboard.getTab("AUTON").add(m_field).withSize(7, 4).withPosition(3, 0);

  }

  @Override
  public void periodic() {
    // Updating the odometry every 20 ms
    odometry.update(
      this.getHeading(),
      encoderTicksToMeters(leftMotors[0].getSelectedSensorPosition()),
      encoderTicksToMeters(rightMotors[0].getSelectedSensorPosition()));
    // differentialDrive.setSafetyEnabled(true);
    // differentialDrive.setSafetyEnabled(false);
    // differentialDrive.setExpiration(.1);
    differentialDrive.feed();

    m_field.setRobotPose(getPose());

  }

  @Override
  public void simulationPeriodic() {
    // Updating the odometry every 20 ms
    odometry.update(
      this.getHeading(),
      encoderTicksToMeters(leftMotors[0].getSelectedSensorPosition()),
      encoderTicksToMeters(rightMotors[0].getSelectedSensorPosition()));
    // differentialDrive.setSafetyEnabled(true);
    // differentialDrive.setSafetyEnabled(false);
    // differentialDrive.setExpiration(.1);
    differentialDrive.feed();

    m_field.setRobotPose(getPose());
  }






  /*
   * This will return the pose of the robot
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void showTraj(String pathName) {
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup(pathName,
    PathPlanner.getConstraintsFromPath(pathName));
    m_field.getObject("Field").setTrajectory(new Trajectory());
    m_field.getObject("Field").setTrajectory(path.get(0));
  }

  public void showTraj() {
    m_field.getObject("Field").setTrajectory(new Trajectory());
  }

  public double encoderTicksToMeters(double currentEncoderValue) {
    return currentEncoderValue/(2048 * 12.75 * Units.inchesToMeters(DriveConstants.kWheelCircumferenceInches));
  }

  /**
   * This will get the wheel speeds for the robot
   * 
   * @return Differential Drive Wheel Speeds
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  /**
   * This method will reset the odometry of the robot
   * 
   * @param Pose2d Need the pose as a parameter
   **/
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(getHeading(),
      encoderTicksToMeters(leftMotors[0].getSelectedSensorPosition()),
      encoderTicksToMeters(rightMotors[0].getSelectedSensorPosition()),
      pose);
  }

  /**
   * This method will set the robot into arcade drive
   * 
   * @param speed    Need the speed of the robot as a speed
   * @param rotation the speed at which the robot will rotate
   **/
  public void arcadeDrive(double speed, double rotation) {
    differentialDrive.arcadeDrive(speed, rotation);
  }

  /**
   * This method will set the robot into tank drive
   * 
   * @param leftTrain  Need the speed of the robot as a speed
   * @param rightTrain the speed at which the robot will rotate
   **/
  public void tankDrive(double leftTrain, double rightTrain) {
    differentialDrive.tankDrive(leftTrain, rightTrain);
    differentialDrive.feed();
  }

  /**
   * This method will set the robot into tank drive
   **/
  public void resetEncoders() {
    leftMotors[0].setSelectedSensorPosition(0);
    leftMotors[1].setSelectedSensorPosition(0);
    rightMotors[0].setSelectedSensorPosition(0);
    rightMotors[1].setSelectedSensorPosition(0);

  }

  public double getRightEncoderVelocity() {
    // Multiply the raw velocity by 10 since it reports per 100 ms, we want the velocity in m/s
    return encoderTicksToMeters(rightMotors[0].getSelectedSensorVelocity() * 10);
}

  public double getLeftEncoderVelocity() {
    return encoderTicksToMeters(leftMotors[0].getSelectedSensorVelocity() * 10) ;
  }

  /**
   * @return Gryo's Angular Velocity/Turn Rate
   **/
  public double getTurnRate() {
    return gyro.getAbsoluteCompassHeading();
  }

  /**
   * Method will get the heading of the robot
   * 
   * @return Rotation2d in degrees
   **/
  public Rotation2d getHeading() {
    double ypr[] = { 0, 0, 0 };
    gyro.getYawPitchRoll(ypr);
    return Rotation2d.fromDegrees((Math.IEEEremainder(ypr[0], 360) * -1.0d));
  }

  public void setPower(double power){

    leftMotors[0].set(ControlMode.PercentOutput, power);
    rightMotors[0].set(ControlMode.PercentOutput, power);
    
}

  public void setPowerLeft(double power){
    leftMotors[0].set(ControlMode.PercentOutput, power);
  }

  public void setPowerRight(double power){
    rightMotors[0].set(ControlMode.PercentOutput, power);
  }

  public void setBreakMode() {
    leftMotors[0].setNeutralMode(NeutralMode.Brake);
    rightMotors[0].setNeutralMode(NeutralMode.Brake);
    leftMotors[1].setNeutralMode(NeutralMode.Brake);
    rightMotors[1].setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Method will get the Gyro's Yaw Rate.
   * 
   * @apiNote Cannot get piegon heading passively in another class. Have to create
   *          a method here
   * @return Gyro Yaw
   **/
  public double getGyroYaw() {
    return gyro.getYaw();
  }

  public double getGyroPitch() {
    return gyro.getPitch();
  }

  public double getGyroRoll() {
    return gyro.getRoll();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors[0].setVoltage(leftVolts);
    rightMotors[0].setVoltage(rightVolts);
    differentialDrive.feed();
  }




  public void resetGyro() {
    gyro.setYaw(0);
  }
  

}
