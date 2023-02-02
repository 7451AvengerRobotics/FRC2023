package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.DriveConstants;




public class Drivetrain extends SubsystemBase {
  protected final WPI_TalonFX[] leftMotors;
  protected final WPI_TalonFX[] rightMotors;
  double kP = 1;
  Pose2d pose;

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.K_TRACK_WIDTH_METERS);
  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(DriveConstants.KS_VOLTS, DriveConstants.KV_VOLT_SECONDS_PER_METER, DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER);
  PIDController leftPIDController = new PIDController(DriveConstants.KP_DRIVE_VELOCITY, 0, 0);
  PIDController rightPIDController = new PIDController(DriveConstants.KP_DRIVE_VELOCITY,0,0);
  private final DifferentialDrive differentialDrive;
  private final Pigeon2 gyro;
  private final DifferentialDriveOdometry odometry;


  /**
   * this method is called when the DriveTrainSubsystem class is initialized.
   */
  public Drivetrain() {
    super();

    this.leftMotors = new WPI_TalonFX[] {
        new WPI_TalonFX(PortConstants.LEFT_DRIVE[0]),
        new WPI_TalonFX(PortConstants.LEFT_DRIVE[1])
    };
    this.rightMotors = new WPI_TalonFX[] {
        new WPI_TalonFX(PortConstants.RIGHT_DRIVE[0]),
        new WPI_TalonFX(PortConstants.RIGHT_DRIVE[1])
    };

    leftMotors[0].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    leftMotors[1].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightMotors[0].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightMotors[1].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);


    differentialDrive = new DifferentialDrive(leftMotors[0], rightMotors[0]);
    gyro = new Pigeon2(PortConstants.Gyro); 

    odometry = new DifferentialDriveOdometry(this.getHeading(), Units.inchesToMeters(DriveConstants.KP_DRIVE_VELOCITY)/60,
    rightMotors[0].getSelectedSensorPosition()/8 * 2 * Math.PI * Units.inchesToMeters(DriveConstants.KP_DRIVE_VELOCITY)/60);




  

    leftMotors[1].follow(leftMotors[0]);
    rightMotors[1].follow(rightMotors[0]);

    rightMotors[0].setInverted(true);
    rightMotors[1].setInverted(InvertType.FollowMaster);

    leftMotors[0].setNeutralMode(NeutralMode.Brake);
    leftMotors[1].setNeutralMode(NeutralMode.Brake);
    rightMotors[0].setNeutralMode(NeutralMode.Brake);
    rightMotors[1].setNeutralMode(NeutralMode.Brake);


  }



  @Override
  public void periodic() {
    odometry.update(this.getHeading(), Units.inchesToMeters(DriveConstants.KP_DRIVE_VELOCITY)/60,
    rightMotors[0].getSelectedSensorPosition()/8 * 2 * Math.PI * Units.inchesToMeters(DriveConstants.KP_DRIVE_VELOCITY)/60);
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftMotors[0].getSelectedSensorVelocity(), rightMotors[0].getSelectedSensorVelocity());
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    odometry.resetPosition(getHeading(), 
    leftMotors[0].getSelectedSensorVelocity(), 
    rightMotors[0].getSelectedSensorVelocity(),
    pose);
  }

  public void arcadeDrive(double speed, double rotation){
    differentialDrive.arcadeDrive(speed, rotation);
  }

  public void tankDrive(double leftTrain, double rightTrain){
    differentialDrive.tankDrive(leftTrain, rightTrain);
    differentialDrive.feed();
  }

  public void resetEncoders(){
    leftMotors[0].setSelectedSensorPosition(0);
    leftMotors[1].setSelectedSensorPosition(0);
    rightMotors[0].setSelectedSensorPosition(0);
    rightMotors[1].setSelectedSensorPosition(0);
    
  }

  public double getTurnRate(){
    return gyro.getAbsoluteCompassHeading();
  }


   public Rotation2d getHeading(){
    double ypr[] = {0, 0, 0};
    gyro.getYawPitchRoll(ypr);
    return Rotation2d.fromDegrees((Math.IEEEremainder(ypr[0], 360)* -1.0d));
  }


}

