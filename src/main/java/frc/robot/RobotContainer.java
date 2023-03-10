// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.SimpleCommands.TurretTestCommand;
import frc.robot.commands.SimpleCommands.VirtualFourBarCommand;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.VirtualFourBar;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.DriveTypes.ArcadeDrive;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  /* Initializing Robot Subsystems */
  private final Drivetrain drivetrain;
//   private final Arm arm;
//   private final Claw claw;
  private final VirtualFourBar bar;
  private final Turret turret;
  private final XboxController controller;
  private final Joystick buttonPanel;

  SendableChooser<Command> chooser = new SendableChooser<>();

/* Initializing Robot Subsystems */

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    drivetrain = new Drivetrain();
    // arm = new Arm();
    // claw = new Claw();
    bar = new VirtualFourBar();
    turret = new Turret();
    controller = new XboxController(ButtonConstants.CONTROLLER_PORT);
    buttonPanel = new Joystick(ButtonConstants.BUTTON_PANEL_PORT);

    configureBindings();
    configureDriveTrain();

    chooser.addOption("Basic Auto", loadPathPlannerTrajectoryToRamseteCommand(
      "C:"+File.separator+"SHISHIR"+File.separator+
      "FRC"+File.separator+"FRC2023"+File.separator+
      "src"+File.separator+"main"+File.separator+
      "deploy"+File.separator+"pathplanner"+File.separator+
      "generatedJSON"+File.separator+"BasicAuto.wpilib.json", 
      true));

    Shuffleboard.getTab("Autonomous").add(chooser);

  }

  public Command loadPathPlannerTrajectoryToRamseteCommand(String filename, boolean resetOdometry){
    Trajectory trajectory;
    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }catch(IOException exception){
      DriverStation.reportError("Unable to open trajectory" + filename, exception.getStackTrace());
      System.out.println("Unable to read from file" + filename);
      return new InstantCommand();
    }

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, drivetrain::getPose, 
        new RamseteController(DriveConstants.K_RAMSETE, DriveConstants.K_RAMSETE_ZETA), 
        new SimpleMotorFeedforward(DriveConstants.KS_VOLTS, DriveConstants.KV_VOLT_SECONDS_PER_METER,
            DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER), 
        DriveConstants.K_DRIVE_KINEMATICS, drivetrain::getWheelSpeeds, 
        new PIDController(DriveConstants.KP_DRIVE_VELOCITY, 0, 0), 
        new PIDController(DriveConstants.KP_DRIVE_VELOCITY, 0, 0), drivetrain::tankDriveVolts, 
        drivetrain);

      if (resetOdometry) {
        return new SequentialCommandGroup(
          new InstantCommand(()->drivetrain.resetOdometry(trajectory.getInitialPose())), ramseteCommand);
      } else{
        return ramseteCommand;
      }

    

  }

  


  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureDriveTrain() {
    // sets the command to drive the robot.
    // will run whenever the drivetrain is not being used.

    drivetrain.setDefaultCommand(
        new ArcadeDrive(drivetrain, controller::getRightY, controller::getRightX, controller::getRightBumper));
  }

  private void configureBindings() {

    /* Button Mapping */

    JoystickButton turnTurretLeft = new JoystickButton(buttonPanel, ButtonConstants.TURN_TURRET_LEFT);
    JoystickButton turnTurretRight = new JoystickButton(buttonPanel, ButtonConstants.TURN_TURRET_RIGHT);

    JoystickButton barUp = new JoystickButton(buttonPanel, ButtonConstants.VBAR_UP);
    JoystickButton barDown = new JoystickButton(buttonPanel, ButtonConstants.VBAR_DOWN);

    // JoystickButton clawIn = new JoystickButton(buttonPanel, ButtonConstants.CLAW_IN);
    // JoystickButton clawOut = new JoystickButton(buttonPanel, ButtonConstants.CLAW_OUT);

    // JoystickButton armExtend = new JoystickButton(buttonPanel, ButtonConstants.ARM_EXTEND);
    // JoystickButton armRetract = new JoystickButton(buttonPanel, ButtonConstants.ARM_RETRACT);
    // JoystickButton armToggle = new JoystickButton(buttonPanel, ButtonConstants.ARM_TOGGLE);

    /* Button Mapping */

    /* Command Mapping */

    turnTurretRight.whileTrue(new TurretTestCommand(turret, 1));
    turnTurretLeft.whileTrue(new TurretTestCommand(turret, -1));

    barUp.whileTrue(new VirtualFourBarCommand(bar, 0.3));
    barDown.whileTrue(new VirtualFourBarCommand(bar, -0.3));

    // clawIn.whileTrue(new ClawCommand(claw, 0.3));
    // clawOut.whileTrue(new ClawCommand(claw, -0.3));

    // armExtend.onTrue(new ArmExtendCommand(arm));
    // armRetract.onTrue(new ArmRetractCommand(arm));
    // armToggle.onTrue(new ArmToggleCommand(arm));

    /* Command Mapping */

  }

  public void robotPeriodic() {
  }

  public Command getAutonomousCommand() {
    
    return chooser.getSelected();

    // var autoVoltageConstraint =
    //   new DifferentialDriveVoltageConstraint(
    //       new SimpleMotorFeedforward(
    //           DriveConstants.KS_VOLTS,
    //           DriveConstants.KV_VOLT_SECONDS_PER_METER,
    //           DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
    //           DriveConstants.K_DRIVE_KINEMATICS,
    //           DriveConstants.MAX_DRIVE_VOLTAGE);

    //   //Create config for trajectory
    //   TrajectoryConfig config =
    //      new TrajectoryConfig(
    //         DriveConstants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED,
    //         DriveConstants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(DriveConstants.K_DRIVE_KINEMATICS)
    //         // Apply the voltage constraint
    //         .addConstraint(autoVoltageConstraint);

    //   // An example trajectory to follow.  All units in meters.
    //   Trajectory exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //       // Start at the origin facing the +X direction
    //       new Pose2d(0, 0, new Rotation2d(0)),
    //       // Pass through these two interior waypoints, making an 's' curve path
    //       List.of(new Translation2d(1, 1), new Translation2d(2, 2)),
    //       // End 3 meters straight ahead of where we started, facing forward
    //       new Pose2d(3, 3, new Rotation2d(0)),
    //       // Pass config
    //       config);

    //     Trajectory trajectory2 =
    //       TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(3, 3, new Rotation2d(-3.14)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(new Translation2d(2, 2), new Translation2d(1, 1)),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         new Pose2d(0, 0, new Rotation2d(-3.14)),
    //         // Pass config
    //         config);

    //   drivetrain.resetOdometry(exampleTrajectory.getInitialPose());
    //   drivetrain.resetGyro();

    //   RamseteCommand ramseteCommand =
    //     new RamseteCommand(
    //       exampleTrajectory,
    //       drivetrain::getPose,
    //       new RamseteController(DriveConstants.K_RAMSETE, DriveConstants.K_RAMSETE_ZETA),
    //       new SimpleMotorFeedforward(
    //         DriveConstants.KS_VOLTS,
    //         DriveConstants.KV_VOLT_SECONDS_PER_METER,
    //         DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
    //         DriveConstants.K_DRIVE_KINEMATICS,
    //         drivetrain::getWheelSpeeds,
    //         new PIDController(DriveConstants.KP_DRIVE_VELOCITY, 0, 0),
    //         new PIDController(DriveConstants.KP_DRIVE_VELOCITY, 0, 0),
    //         drivetrain::tankDriveVolts,
    //         drivetrain);
    //   RamseteCommand ramseteCommand1 =
    //     new RamseteCommand(
    //     trajectory2,
    //     drivetrain::getPose,
    //     new RamseteController(DriveConstants.K_RAMSETE, DriveConstants.K_RAMSETE_ZETA),
    //     new SimpleMotorFeedforward(
    //       DriveConstants.KS_VOLTS,
    //       DriveConstants.KV_VOLT_SECONDS_PER_METER,
    //       DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
    //       DriveConstants.K_DRIVE_KINEMATICS,
    //       drivetrain::getWheelSpeeds,
    //       new PIDController(DriveConstants.KP_DRIVE_VELOCITY, 0, 0),
    //       new PIDController(DriveConstants.KP_DRIVE_VELOCITY, 0, 0),
    //       drivetrain::tankDriveVolts,
    //       drivetrain);
        


    // return new SequentialCommandGroup(ramseteCommand,
    //                                   new RunCommand(() -> drivetrain.tankDriveVolts(0, 0)),
    //                                   ramseteCommand1,
    //                                   new RunCommand(() -> drivetrain.tankDriveVolts(0, 0))
    //                                   );  }

    
}
}
