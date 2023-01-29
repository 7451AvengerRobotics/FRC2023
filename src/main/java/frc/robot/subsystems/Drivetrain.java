package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.InvertType;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;




public class Drivetrain extends SubsystemBase {


  // initializing motors
  // protected final WPI_TalonFX[] leftMotors;
  // protected final WPI_TalonFX[] rightMotors;
   protected final WPI_TalonFX newMotor;

  protected final WPI_TalonFX leftMaster;
  protected final WPI_TalonFX rightMaster;
  protected final WPI_TalonFX leftSlave;
  protected final WPI_TalonFX rightSlave;

  //unit conversion from ticks to feet
  private final double kDriveTick2Feet = (1.0/2048) * 6 * Math.PI/12; //Need to change values based on the gear ratio on the drivetrain

  Pose2d pose;

  protected final DifferentialDrive driveTrain;
  private final Pigeon2 gyro = new Pigeon2(5);

  


  /**
   * this method is called when the DriveTrainSubsystem class is initialized.
   */
  public Drivetrain() {
    super();

    leftMaster = new WPI_TalonFX(PortConstants.LEFT_DRIVE[0]);
    rightMaster = new WPI_TalonFX(PortConstants.RIGHT_DRIVE[0]);
    

    leftSlave = new WPI_TalonFX(PortConstants.LEFT_DRIVE[1]);
    rightSlave = new WPI_TalonFX(PortConstants.RIGHT_DRIVE[1]);


    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);


    // this.leftMotors = new WPI_TalonFX[] {
    //     new WPI_TalonFX(PortConstants.LEFT_DRIVE[0]),
    //     new WPI_TalonFX(PortConstants.LEFT_DRIVE[1])
    // };
    // this.rightMotors = new WPI_TalonFX[] {
    //     new WPI_TalonFX(PortConstants.RIGHT_DRIVE[0]),
    //     new WPI_TalonFX(PortConstants.RIGHT_DRIVE[1])
    // };

    newMotor = new WPI_TalonFX(2);

    // leftMotors[0].configFactoryDefault();
    // leftMotors[1].configFactoryDefault();
    // rightMotors[0].configFactoryDefault();
    // rightMotors[1].configFactoryDefault();


    //Initializing Encoders  
    newMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    
    //Setting whats + or - for encoders
    leftMaster.setSensorPhase(false); //Need to change values based on drivetrain
    rightMaster.setSensorPhase(false);

    //Reseting Encoder Values
    leftMaster.setSelectedSensorPosition(0, 0, 10);
    rightMaster.setSelectedSensorPosition(0, 0, 10);



    /*leftMotors[0].configOpenloopRamp(0);
    leftMotors[1].configOpenloopRamp(0);
    rightMotors[0].configOpenloopRamp(0);
    rightMotors[1].configOpenloopRamp(0);*/

    // leftMotors[1].follow(leftMotors[0]);
    // rightMotors[1].follow(rightMotors[0]);

    // rightMotors[0].setInverted(true);
    // rightMotors[1].setInverted(InvertType.FollowMaster);

    // leftMotors[0].setNeutralMode(NeutralMode.Brake);
    // leftMotors[1].setNeutralMode(NeutralMode.Brake);
    // rightMotors[0].setNeutralMode(NeutralMode.Brake);
    // rightMotors[1].setNeutralMode(NeutralMode.Brake);

    driveTrain = new DifferentialDrive(
      leftMaster,
      rightMaster);
      

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
  }

  /**
  * Returns the currently-estimated pose of the robot.

  * @return The pose.
  */
  /*public Pose2d getPose() {
    //return pose
    return odometry.getPoseMeters();
  }*/


  /**
   * Returns the current wheel speeds of the robot.

   * @return The current wheel speeds.
   */
  // public DifferentialDriveWheelSpeeds getWheelSpeeds() {
  //   return new DifferentialDriveWheelSpeeds(
  //       leftMotors[0].getSelectedSensorVelocity()/5.95 * 2 * Math.PI * Units.inchesToMeters(3)/60,
  //       rightMotors[0].getSelectedSensorVelocity()/5.95 * 2 * Math.PI * Units.inchesToMeters(3)/60
  //     );
  // }


  /**
   * Resets the odometry to the specified pose.

   * @param pose The pose to which to set the odometry.
   */
  /*public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }*/

  /**
   * Controls the left and right sides of the drive directly with voltages.

   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  // public void tankDriveVolts(double leftVolts, double rightVolts) {
  //   leftMotors[0].setVoltage(leftVolts/12);
  //   rightMotors[0].setVoltage(rightVolts/12);
  //   driveTrain.feed();
  // }

  /**
   * just call the arcadedrive method with a differential drive.
   */
  public void arcadeDrive(double speed, double rotation) {
    driveTrain.arcadeDrive(speed, rotation);
  }

  public void tankDrive(double leftTrain, double rightTrain)  {
    driveTrain.tankDrive(leftTrain, rightTrain);
  }

  /**
   * Curvature drive method for differential drivetrain.

   * The rotation argument controls the curvature of the robot's path rather thanS
   * its rate of heading change. This makes the robot more controllable at high
   * speeds.
   */
  public void curvatureDrive(double speed, double rotation, boolean turn) {
    driveTrain.curvatureDrive(speed, rotation, turn);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  // public void resetEncoders() {
  //   leftMotors[0].setSelectedSensorPosition(0);
  //   leftMotors[1].setSelectedSensorPosition(0);
  //   rightMotors[0].setSelectedSensorPosition(0);
  //   rightMotors[1].setSelectedSensorPosition(0);
  // }

  /**
   * Gets the average distance of the two encoders.

   * @return the average of the two encoder readings
   */
  // public double getAverageEncoderDistance() {
  //   return (
  //       leftMotors[0].getSelectedSensorPosition()
  //       + leftMotors[1].getSelectedSensorPosition()
  //       + rightMotors[0].getSelectedSensorPosition() * -1
  //       + rightMotors[1].getSelectedSensorPosition() * -1
  //     ) / 4;
  // }

  public void gyroCalibrate() {
  }

  public void zeroHeading() {

    }


    public double getLeftEncoderValue(){
      return leftMaster.getSelectedSensorPosition() * kDriveTick2Feet;
    }

    public double getRightEncoderValue(){
      return rightMaster.getSelectedSensorPosition() * kDriveTick2Feet;
    }
  

  // public void setOutput(double leftVolts, double rightVolts) {
  //   leftMotors[0].set(leftVolts/12);
  //   rightMotors[0].set(rightVolts/12);
  // }

  public void teleopPeriodic(){
      int loopCount = 0;
      if(loopCount++ >10){
        loopCount = 0;
        double yaw = gyro.getYaw();
        SmartDashboard.putNumber("DriveTrainPower", yaw);
      }

  }

  }





