// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoCommands.DriveBackAuto;
import frc.robot.commands.DriveTypes.TankDrive;
import frc.robot.commands.SimpleCommands.TurretTestCommand;
import frc.robot.commands.SimpleCommands.VFBAREncoder;
import frc.robot.commands.SimpleCommands.VirtualFourBarCommand;
import frc.robot.commands.SimpleCommands.ArmCommands.ArmExtendCommand;
import frc.robot.commands.SimpleCommands.ArmCommands.ArmToggleCommand;
import frc.robot.commands.SimpleCommands.ClawCommands.ClawIntake;
import frc.robot.commands.SimpleCommands.ClawCommands.ClawOuttake;
import frc.robot.commands.SimpleCommands.ClawCommands.ClawToggle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.AutoBalance;
import frc.robot.subsystems.Claw;
//import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.VirtualFourBar;

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
  public final  Drivetrain drivetrain;
  private final Arm arm;
  private final Claw claw;
  private final VirtualFourBar bar;
  private final Turret turret;
  private final XboxController controller;
  private final Joystick buttonPanel;

  

/* Initializing Robot Subsystems */

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * 
   * 
   */
  SendableChooser<Command> chooser = new SendableChooser<>();
  public static HashMap<Command, String> autoMap = new HashMap<>();

  public Command ramAutoBuilder(String pathName, HashMap<String, Command> eventMap) {

    RamseteAutoBuilder pathBuilder = new RamseteAutoBuilder(drivetrain::getPose, drivetrain::resetOdometry, 
      new RamseteController(DriveConstants.K_RAMSETE, DriveConstants.K_RAMSETE_ZETA), DriveConstants.K_DRIVE_KINEMATICS, 
      new SimpleMotorFeedforward(DriveConstants.KS_VOLTS, DriveConstants.KV_VOLT_SECONDS_PER_METER, DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER), 
      drivetrain::getWheelSpeeds, new PIDConstants(DriveConstants.KP_DRIVE_VELOCITY, 0, 0), 
      drivetrain::tankDriveVolts, eventMap, true, drivetrain);

    List<PathPlannerTrajectory> pathToFollow = PathPlanner.loadPathGroup(pathName, PathPlanner.getConstraintsFromPath(pathName));
    final Command auto = pathBuilder.fullAuto(pathToFollow);
    autoMap.put(auto, pathName);
    return auto;
  }


  
  public RobotContainer() {
    // Configure the trigger bindings
    drivetrain = new Drivetrain();
    arm = new Arm();
    claw = new Claw();
    bar = new VirtualFourBar();
    turret = new Turret();
    controller = new XboxController(ButtonConstants.CONTROLLER_PORT);
    buttonPanel = new Joystick(ButtonConstants.BUTTON_PANEL_PORT);

    setBasicChargeAutoMap();
    configureBindings();
    configureDriveTrain();
    getAutonomousCommand();

    chooser.setDefaultOption("Balance Auto Inside", ramAutoBuilder("BasicChargeAuto", AutoConstants.basicChargeAuto));
    
  }

  public void setBasicChargeAutoMap() {
    AutoConstants.basicChargeAuto.put("spit", new ClawOuttake(claw, 0.5).withTimeout(2));
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


    // drivetrain.setDefaultCommand(
    //     new ArcadeDrive(
    //         drivetrain,
    //         controller::getLeftY,
    //         controller::getRightX,
    //         controller::getRightBumper,
    //         controller:: getLeftBumper));
    

  drivetrain.setDefaultCommand(
    new TankDrive(
        drivetrain,
        controller::getLeftY,
        controller::getRightY,
        controller::getRightBumper));
}



  private void configureBindings() {

    /* Button Mapping */

    JoystickButton turnTurretLeft = new JoystickButton(buttonPanel, ButtonConstants.TURN_TURRET_LEFT);
    JoystickButton turnTurretRight = new JoystickButton(buttonPanel, ButtonConstants.TURN_TURRET_RIGHT);

    JoystickButton barUp = new JoystickButton(buttonPanel, ButtonConstants.VBAR_UP);
    JoystickButton barDown = new JoystickButton(buttonPanel, ButtonConstants.VBAR_DOWN);

    JoystickButton clawIn = new JoystickButton(buttonPanel, ButtonConstants.CLAW_IN);
    JoystickButton clawOut = new JoystickButton(buttonPanel, ButtonConstants.CLAW_OUT);

    JoystickButton armToggle = new JoystickButton(buttonPanel, ButtonConstants.ARM_TOGGLE);
    JoystickButton clawToggle = new JoystickButton(buttonPanel, ButtonConstants.CLAW_TOGGLE);

    JoystickButton armWitEncoder = new JoystickButton(buttonPanel, 10);
    /* Button Mapping */

    /* Command Mapping */

    turnTurretRight.whileTrue(new TurretTestCommand(turret, 1));
    turnTurretLeft.whileTrue(new TurretTestCommand(turret, -1));

    barUp.whileTrue(new VirtualFourBarCommand(bar, arm, -0.3));
    barDown.whileTrue(new VirtualFourBarCommand(bar, arm, 0.3));

    clawIn.whileTrue(new ClawIntake(claw, 1));
    clawOut.whileTrue(new ClawIntake(claw, -1));
    clawToggle.onTrue(new ClawToggle(claw));

    armToggle.whileTrue(new ArmToggleCommand(arm));

    armWitEncoder.whileTrue(new VFBAREncoder(bar, arm, 67169));
    //31714 is for cone high
    //1813 is for cube high
    //13393 for cube mid



    
    /* Command Mapping */

  }

  public void robotPeriodic() {
  }

  public Command getAutonomousCommand() {

    new SequentialCommandGroup(   
      new ClawOuttake(claw, 0.5).withTimeout(2),
      new WaitCommand(1),
      new DriveBackAuto(drivetrain, 0.5).withTimeout(1).withTimeout(2));
      //new AutoBalance();

   return null;
    
}
}
