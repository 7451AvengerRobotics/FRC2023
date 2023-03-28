// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.commands.AutoCommands.ComplexAuto;
import frc.robot.commands.AutoCommands.Encoder1Auto;
import frc.robot.commands.AutoCommands.Encoder2Auto;
import frc.robot.commands.DriveTypes.AlexDrive;
import frc.robot.commands.SimpleCommands.SolenoidCommand;
import frc.robot.commands.SimpleCommands.TurretTestCommand;
import frc.robot.commands.SimpleCommands.ClawCommands.ClawIntake;
import frc.robot.commands.SimpleCommands.ClawCommands.ClawOuttake;
import frc.robot.commands.SimpleCommands.ClawCommands.ClawToggle;
import frc.robot.commands.SimpleCommands.VirtualFourBar.EncoderandArm;
import frc.robot.commands.SimpleCommands.VirtualFourBar.ResetVFbarEncoder;
import frc.robot.commands.SimpleCommands.VirtualFourBar.StandardEncoder;
import frc.robot.commands.SimpleCommands.VirtualFourBar.VirtualFourBarCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.VirtualFourBar;

public class RobotContainer {

  /* Initializing Robot Subsystems */
  private final  Drivetrain drivetrain;
  private final Arm arm;
  private final Pigeon2 gyro;
  private final Claw claw;
  private final VirtualFourBar bar;
  private final Turret turret;
  private final PS4Controller controller;
  private final Joystick buttonPanel;
  private final Trigger rightBumper;
  

/* Initializing Robot Subsystems */

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * 
   * 
   * 
   */




  public static SendableChooser<Command> chooser = new SendableChooser<>();
  public static HashMap<Command, String> autoMap = new HashMap<>();

  public Command ramAutoBuilder(String pathName, HashMap<String, Command> eventMap) {

    RamseteAutoBuilder pathBuilder = new RamseteAutoBuilder(drivetrain::getPose, drivetrain::resetOdometry, 
      new RamseteController(DriveConstants.K_RAMSETE, DriveConstants.K_RAMSETE_ZETA), DriveConstants.K_DRIVE_KINEMATICS, 
      new SimpleMotorFeedforward(DriveConstants.KS_VOLTS, DriveConstants.KV_VOLT_SECONDS_PER_METER, DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER), 
      drivetrain::getWheelSpeeds, new PIDConstants(DriveConstants.KP_DRIVE_VELOCITY, 0, 0), 
      drivetrain::tankDriveVolts, eventMap, true, drivetrain);

    List<PathPlannerTrajectory> pathToFollow = PathPlanner.loadPathGroup(pathName,
          PathPlanner.getConstraintsFromPath(pathName));
    final Command auto = pathBuilder.fullAuto(pathToFollow);
    autoMap.put(auto, pathName);
    return auto;
  }

    /**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

  public RobotContainer() {
    // Configure the trigger bindings
    drivetrain = new Drivetrain();
    arm = new Arm();
    claw = new Claw();
    bar = new VirtualFourBar();
    gyro = new Pigeon2(PortConstants.Gyro);
    turret = new Turret();
    controller = new PS4Controller(ButtonConstants.CONTROLLER_PORT);
    buttonPanel = new Joystick(ButtonConstants.BUTTON_PANEL_PORT);
    rightBumper = new JoystickButton(controller, PS4Controller.Button.kCross.value);

  
    configureBindings();
    configureDriveTrain();
    getAutonomousCommand();

    setBasicChargeAutoMap();
    setTwoCubeAuto();

    Shuffleboard.getTab("Auton").add(chooser).withSize(3, 1);
    Command instantCmd = new InstantCommand();
    chooser.setDefaultOption("Nothing", instantCmd);
    autoMap.put(instantCmd, "nothing");
    chooser.addOption("Community With Cube Leave", new ComplexAuto(arm, drivetrain, -0.3,claw, -0.3, turret, bar));
    chooser.addOption("HighCubeNoLeave", new Encoder2Auto(arm, drivetrain, -0.3,bar,claw, -1, turret, gyro));
    chooser.addOption("Balance Auto With Gyro", new Encoder1Auto(arm, drivetrain, -0.3,bar,claw, -0.25, turret, gyro));




    chooser.addOption("2CubeAuto", ramAutoBuilder("2CubeAuto", AutoConstants.twoCubeAuto));
    chooser.addOption("Cube and park", ramAutoBuilder("BasicChargeAuto", AutoConstants.basicChargeAuto));


  }

  public void setBasicChargeAutoMap() {
    AutoConstants.basicChargeAuto.put("Start", new ClawOuttake(claw, 0.5).withTimeout(2));
    AutoConstants.basicChargeAuto.put("Stop", new ClawToggle(claw));
  }

  public void setTwoCubeAuto() {
    AutoConstants.twoCubeAuto.put("Start", new SequentialCommandGroup(
    new ClawOuttake(claw, 0.5).withTimeout(1)));
    AutoConstants.twoCubeAuto.put("IntakeArm", new ParallelCommandGroup(
    new TurretTestCommand(turret, 0.3).withTimeout(1), 
    new VirtualFourBarCommand(bar, arm, -0.3).withTimeout(1.3)));
    AutoConstants.twoCubeAuto.put("Intake", new SequentialCommandGroup(
    new ClawIntake(claw, 0.5), 
    new VirtualFourBarCommand(bar, arm, 0.3).withTimeout(2)));
    AutoConstants.twoCubeAuto.put("TurretFlip", new TurretTestCommand(turret, -0.3).withTimeout(1));
    AutoConstants.twoCubeAuto.put("Stop", new SequentialCommandGroup(
    new VirtualFourBarCommand(bar, arm, -0.3).withTimeout(0.5),
    new ClawOuttake(claw, 0.5).withTimeout(1)));
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

      // drivetrain.setDefaultCommand(
      //   new ArcadeDrive(
      //       drivetrain,
      //       arm,
      //       controller::getLeftY,
      //       controller::getRightX,
      //       controller:: getLeftBumper));


            drivetrain.setDefaultCommand( 
              new AlexDrive( 
              drivetrain, 
              controller::getR2Axis, 
              controller::getL2Axis, 
              controller::getLeftX, 
              controller::getL1Button, 
              controller::getR1Button));
}



  private void configureBindings() {

       /* Button Mapping */


    /* Actual Buttons */

    JoystickButton midCone = new JoystickButton(buttonPanel, ButtonConstants.MidCone);
    JoystickButton midCube = new JoystickButton(buttonPanel, ButtonConstants.MidCube);

    JoystickButton highCube = new JoystickButton(buttonPanel, ButtonConstants.HighCube);
    JoystickButton grabObject = new JoystickButton(buttonPanel, ButtonConstants.Ground);
    JoystickButton resetBar = new JoystickButton(buttonPanel, ButtonConstants.ResetEncoder);

    JoystickButton clawIntake = new JoystickButton(buttonPanel, ButtonConstants.ClawIntake);
    JoystickButton clawOuttake = new JoystickButton(buttonPanel, ButtonConstants.ClawOuttake);
    JoystickButton clawToggle = new JoystickButton(buttonPanel, ButtonConstants.CLAW_TOGGLE);

    JoystickButton turretRight = new JoystickButton(buttonPanel, ButtonConstants.TurretLeft);
    JoystickButton turretLeft = new JoystickButton(buttonPanel, ButtonConstants.TurretRight);

    JoystickButton lockSolenoid = new JoystickButton(buttonPanel, ButtonConstants.lockSolenoid);

    

    /* Actual Buttons */




    /*Actual Command Mapping */
   midCone.onTrue(new EncoderandArm(bar, arm, 58464)); // 5
   midCube.onTrue(new StandardEncoder(bar, arm, 13800)); // 6


    highCube.onTrue(new EncoderandArm(bar, arm, 30786)); // 2
    grabObject.onTrue(new StandardEncoder(bar, arm, 69900)); // 1
    resetBar.onTrue(new ResetVFbarEncoder(bar, arm, 0)); // 11


    clawIntake.whileTrue(new ClawIntake(claw, 1)); // 3
    clawOuttake.whileTrue(new ClawOuttake(claw, -0.7)); // 4
    clawToggle.whileTrue(new EncoderandArm(bar, arm, 35000)); // 8

    lockSolenoid.onTrue(new SolenoidCommand(arm)); // 7


    
    turretRight.whileTrue(new TurretTestCommand(turret, 0.5));
    turretLeft.whileTrue(new TurretTestCommand(turret, -0.5));
    rightBumper.whileTrue(new ClawToggle(claw));   /*Actual Command Mapping */








    /*    TestButton Mapping */
    // JoystickButton clawToggle = new JoystickButton(buttonPanel, ButtonConstants.clawToggle);
    // JoystickButton armToggle = new JoystickButton(buttonPanel, ButtonConstants.armToggle);
    // JoystickButton clawOut = new JoystickButton(buttonPanel, ButtonConstants.clawOut);
    // JoystickButton clawIn = new JoystickButton(buttonPanel, ButtonConstants.clawIn);
    // JoystickButton vfBarD = new JoystickButton(buttonPanel, ButtonConstants.vfbarDown);
    // JoystickButton vfBarUp = new JoystickButton(buttonPanel, ButtonConstants.vfbarUp);
    // JoystickButton lockSolenoid = new JoystickButton(buttonPanel, 7);
    // JoystickButton turretL = new JoystickButton(buttonPanel, ButtonConstants.turretL);
    // JoystickButton turretR = new JoystickButton(buttonPanel, 11);
    // JoystickButton midCone = new JoystickButton(buttonPanel, 9);
    /* TestButton Mapping */


    /* Test Mapping */
    // clawToggle.onTrue(new ClawToggle(claw));
    // armToggle.onTrue(new ArmToggleCommand(arm));
    // clawOut.whileTrue(new ClawOuttake(claw, -1));
    // clawIn.whileTrue(new ClawIntake(claw, 1));
    // vfBarD.whileTrue(new VirtualFourBarCommand(bar, arm, 0.3));
    // vfBarUp.whileTrue(new VirtualFourBarCommand(bar, arm, -0.3));
    // lockSolenoid.onTrue(new SolenoidCommand(arm));
    // turretL.whileTrue(new MidConeCommand(bar, arm, 30786));
    // turretR.whileTrue(new VFBAREncoder(bar, arm, 0));
    // midCone.whileTrue(new MidConeCommand(bar, arm, 62464));
    //turretR.whileTrue(new TurretTestCommand(turret, 0.-2));
    /* Test Mapping */

    /* Command Mapping */

  }

  public void robotPeriodic() {
 
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
}
}
