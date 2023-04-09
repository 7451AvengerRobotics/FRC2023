// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.VirtualFourBar;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Drivetrain drivetrain;
  private Turret turret;
  private VirtualFourBar bar;
  private ColorSensor color;  
  private Led led;
  public Field2d m_field = new Field2d();
  ShuffleboardTab autoTab = Shuffleboard.getTab("AUTON");
  GenericEntry allianceColor = 
    autoTab.add("Alliance", true)
    .withProperties(Map.of("colorWhenTrue", "blue"))
    .withPosition(0,1)
    .withSize(3,1)
    .getEntry();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    drivetrain = new Drivetrain();
    led = new Led();
    turret = new Turret();
    bar = new VirtualFourBar(); 
    color = new ColorSensor();
    turret.zeroSensors();
    bar.zeroSensors();  
    CameraServer.startAutomaticCapture();

    Shuffleboard.getTab("AUTON").add(m_field).withSize(7, 4).withPosition(3, 0);
    m_field.setRobotPose(drivetrain.getPose());

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.


    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Yaw", drivetrain.getGyroYaw());
    SmartDashboard.putNumber("Pitch", drivetrain.getGyroPitch());
    SmartDashboard.putNumber("Roll", drivetrain.getGyroRoll());
    SmartDashboard.putString("Rotation2d", toString());
    SmartDashboard.putNumber("Turret Encoder Position", turret.getencoderValues());
    SmartDashboard.putNumber("Mini-Arm Encoder Pos", bar.getencoderValues());
    SmartDashboard.putNumber("ColorSensor Value ", color.getProximity()); 
    SmartDashboard.putNumber("ColorSensor Color ", color.detectColor());
    SmartDashboard.putBoolean("Dected Object? ", color.detectObject());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // if (RobotContainer.autoMap.get(RobotContainer.chooser.getSelected()) != "nothing" && RobotContainer.autoMap.get(RobotContainer.chooser.getSelected()) != "test") {
    //   drivetrain.showTraj(RobotContainer.autoMap.get(RobotContainer.chooser.getSelected()));
    // } else {
    //   drivetrain.showTraj();
    // }

    // if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
    //   allianceColor.setBoolean(true);
    // } else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
    //   allianceColor.setBoolean(false);
    // } else {
    //   allianceColor.setString("error");
    }


  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // drivetrain.setBreakMode();
    // drivetrain.resetEncoders();
    // drivetrain.resetGyro();
    // drivetrain.resetOdometry(new Pose2d(0,0, new Rotation2d()));
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
       m_autonomousCommand.schedule();
    }
        
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    led.setColor(255, 0, 0);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }


  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}