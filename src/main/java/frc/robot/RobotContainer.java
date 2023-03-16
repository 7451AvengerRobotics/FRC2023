// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoCommands.BalanceCommand;
import frc.robot.commands.AutoCommands.ComplexAuto;
import frc.robot.commands.AutoCommands.GetOnRamp;
import frc.robot.commands.DriveTypes.ArcadeDrive;
import frc.robot.commands.DriveTypes.TankDrive;
import frc.robot.commands.SimpleCommands.TurretTestCommand;
import frc.robot.commands.SimpleCommands.VFBAREncoder;
import frc.robot.commands.SimpleCommands.VirtualFourBarCommand;
import frc.robot.commands.SimpleCommands.ClawCommands.ClawIntake;
import frc.robot.commands.SimpleCommands.ClawCommands.ClawOuttake;
import frc.robot.commands.SimpleCommands.ClawCommands.ClawToggle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
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
  private Boolean driveState;

  

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
    driveState = false;

    setBasicChargeAutoMap();
    configureBindings();
    configureDriveTrain();
    getAutonomousCommand();

    chooser.addOption("Balance Auto Shishir", ramAutoBuilder("BasicChargeAuto", AutoConstants.basicChargeAuto));
    chooser.setDefaultOption("Balance Auto Timed", new ComplexAuto(arm, drivetrain, 0.5,claw, 0.5));

  }

  public void setBasicChargeAutoMap() {
    AutoConstants.basicChargeAuto.put("Start", new ClawOuttake(claw, 0.5).withTimeout(2));
    AutoConstants.basicChargeAuto.put("Stop", new SequentialCommandGroup(new GetOnRamp(drivetrain), new BalanceCommand(0.39)));
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

/*
Sets the state of the type of drivetrain. For example if driver wants Arcade than the driver presses the A button to enter arcade mode.
If the driver presses the B button than the drivtrain will reset back to Tank Drive
*/
    if(controller.getAButtonPressed()){
      driveState = true;
    }

    if(controller.getBButtonPressed()){
      driveState = false;
    }

    if(driveState == true){
      drivetrain.setDefaultCommand(
        new ArcadeDrive(
            drivetrain,
            arm,
            controller::getLeftY,
            controller::getRightX,
            controller:: getLeftBumper));
    }else{
      drivetrain.setDefaultCommand(
        new TankDrive(
          drivetrain,
          arm,
          controller::getLeftY,
          controller::getRightY,
          controller::getRightBumper));
    }
}



  private void configureBindings() {

    /* Button Mapping */

    JoystickButton groundState = new JoystickButton(buttonPanel, ButtonConstants.Ground);
    JoystickButton MidCube = new JoystickButton(buttonPanel, ButtonConstants.MidCube);
    JoystickButton MidCone = new JoystickButton(buttonPanel, ButtonConstants.MidCone);
    JoystickButton HighCube = new JoystickButton(buttonPanel, ButtonConstants.HighCube);
    JoystickButton HighCone = new JoystickButton(buttonPanel, ButtonConstants.HighCone);
    JoystickButton ResetEncoder = new JoystickButton(buttonPanel, ButtonConstants.ResetEncoder);
    JoystickButton clawToggle = new JoystickButton(buttonPanel, ButtonConstants.CLAW_TOGGLE);
    JoystickButton clawIn = new JoystickButton(buttonPanel, ButtonConstants.ClawIntake);
    JoystickButton clawOut = new JoystickButton(buttonPanel, ButtonConstants.ClawOuttake);
    JoystickButton turretLeft = new JoystickButton(buttonPanel, ButtonConstants.TurretLeft);
    JoystickButton turretRight = new JoystickButton(buttonPanel, ButtonConstants.TurretRight);


    /* Command Mapping */
    MidCone.whileTrue(new VirtualFourBarCommand(bar, arm, -0.3)); //2
    MidCube.onTrue(new VFBAREncoder(bar, arm, 30786)); //4

    HighCube.onTrue(new VFBAREncoder(bar, arm, 40000)); //3
    HighCone.onTrue(new VFBAREncoder(bar, arm, 40000));
    groundState.onTrue(new VFBAREncoder(bar, arm, 68027));
    ResetEncoder.onTrue(new VFBAREncoder(bar, arm, 0)); //5

    clawIn.whileTrue(new ClawIntake(claw, 1)); //9
    clawOut.whileTrue(new ClawOuttake(claw, -1)); //10
    clawToggle.whileTrue(new ClawToggle(claw));

    turretRight.whileTrue(new TurretTestCommand(turret, 0.3));
    turretLeft.whileTrue(new TurretTestCommand(turret, -0.3));

  }

  public void robotPeriodic() {
  }

  public Command getAutonomousCommand() {

    return chooser.getSelected();

}
}
