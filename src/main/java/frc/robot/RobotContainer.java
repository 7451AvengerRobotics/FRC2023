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
import frc.robot.commands.AutoCommands.CommunityWithCode;
import frc.robot.commands.AutoCommands.Encoder1Auto;
import frc.robot.commands.AutoCommands.Encoder2Auto;
import frc.robot.commands.AutoCommands.TwoElementAuto;
import frc.robot.commands.DriveTypes.ArcadeDrive;
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
import frc.robot.subsystems.ColorSensor;
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
  private final ColorSensor color;
  private final Trigger rightBumper;
  private final Trigger leftTrigger;
  private final Trigger rightTrigger;


  

/* Initializing Robot Subsystems */

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * 
   * 
   * 
   */




 
  public static HashMap<Command, String> autoMap = new HashMap<>();

  public Command ramAutoBuilder(String pathName, HashMap<String, Command> eventMap) {

    RamseteAutoBuilder pathBuilder = new RamseteAutoBuilder(
      drivetrain::getPose, 
      drivetrain::resetOdometry, 
      new RamseteController(DriveConstants.K_RAMSETE, DriveConstants.K_RAMSETE_ZETA), 
      DriveConstants.K_DRIVE_KINEMATICS, 
      new SimpleMotorFeedforward(
        DriveConstants.KS_VOLTS, 
        DriveConstants.KV_VOLT_SECONDS_PER_METER, 
        DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER), 
      drivetrain::getWheelSpeeds, 
      new PIDConstants(DriveConstants.KP_DRIVE_VELOCITY, 0, 0), 
      drivetrain::tankDriveVolts, 
      eventMap, 
      true, 
      drivetrain);

    List<PathPlannerTrajectory> pathToFollow = PathPlanner.loadPathGroup(pathName,
          PathPlanner.getConstraintsFromPath(pathName));
    final Command auto = pathBuilder.fullAuto(pathToFollow);
    autoMap.put(auto, pathName);
    return auto;
  }

  public static SendableChooser<Command> chooser = new SendableChooser<>();

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
    color = new ColorSensor();
    gyro = new Pigeon2(PortConstants.Gyro);
    turret = new Turret();
    controller = new PS4Controller(ButtonConstants.CONTROLLER_PORT);
    buttonPanel = new Joystick(ButtonConstants.BUTTON_PANEL_PORT);
    rightBumper = new JoystickButton(controller, PS4Controller.Button.kCross.value);
    leftTrigger = new JoystickButton(controller, PS4Controller.Button.kL2.value);
    rightTrigger = new JoystickButton(controller, PS4Controller.Button.kR2.value);


  
    configureBindings();
    configureDriveTrain();
    getAutonomousCommand();

    //setBasicChargeAutoMap();
    setTwoCubeAuto();

    Shuffleboard.getTab("AUTON").add(chooser).withSize(3, 1);
    Command instantCmd = new InstantCommand();
    chooser.setDefaultOption("Nothing", instantCmd);
    autoMap.put(instantCmd, "nothing");
    //chooser.addOption("Balance Auto Timed", new ComplexAuto(arm, drivetrain, -0.3,claw, -0.8, turret, bar));
    chooser.addOption("2CubeAuto", ramAutoBuilder("2CubeAuto", AutoConstants.twoCubeAuto));
    chooser.addOption("BalanceChargePath", ramAutoBuilder("BasicChargeAuto", AutoConstants.basicChargeAuto));
    //chooser.addOption("God I hope this works", new EncoderAuto(arm, drivetrain, -0.3, claw, -0.8, turret, bar));
    chooser.addOption("God I hope this work", new TwoElementAuto(arm, drivetrain, -0.3, claw, -0.5, turret, bar));
    chooser.addOption("One Cube and Park", new Encoder1Auto(arm, drivetrain, -0.3, bar, claw, -0.2, turret, gyro));
    chooser.addOption("One Cube and ParkPLS", new Encoder2Auto(arm, drivetrain, -0.3, bar, claw, -0.3, turret, gyro));
    chooser.addOption("Auto Encoder", new TwoElementAuto(arm, drivetrain, -0.3, claw, -0.5, turret, bar));
    chooser.addOption("Community w Cube Leave", new CommunityWithCode(arm, drivetrain, -0.3, bar, claw, -0.3, turret, gyro));



  }

  public void setBasicChargeAutoMap() {
    AutoConstants.basicChargeAuto.put("Start", new ClawOuttake(claw, 0.5).withTimeout(2));
    AutoConstants.basicChargeAuto.put("Stop", new ClawToggle(claw));
  }

  public void setTwoCubeAuto() {
    AutoConstants.twoCubeAuto.put("Start", new SequentialCommandGroup(
      new ClawOuttake(claw, 0.5).withTimeout(0.5)));
      AutoConstants.twoCubeAuto.put("IntakeArm", new ParallelCommandGroup(
      new TurretTestCommand(turret, 0.3).withTimeout(1), 
      new VirtualFourBarCommand(bar, arm, -0.3).withTimeout(1.3)));
    AutoConstants.twoCubeAuto.put("Intake", new SequentialCommandGroup(
      new ClawIntake(claw, 1), 
      new VirtualFourBarCommand(bar, arm, 0.3).withTimeout(1)));
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

      drivetrain.setDefaultCommand(
        new ArcadeDrive(
            drivetrain,
            arm,
            controller::getLeftY,
            controller::getRightX,
            controller:: getR1Button));
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
   midCone.onTrue(new EncoderandArm(bar, arm, 61000)); // 5
   midCube.onTrue(new StandardEncoder(bar, arm, 13500)); // 6


    highCube.onTrue(new EncoderandArm(bar, arm, 37786)); // 2
    grabObject.onTrue(new StandardEncoder(bar, arm, 77347)); // 1
    resetBar.onTrue(new ResetVFbarEncoder(bar, arm, 0)); // 11


    clawIntake.whileTrue(new ClawIntake(claw, 1)); // 3
    clawOuttake.whileTrue(new ClawOuttake(claw, -1)); // 4
    clawToggle.whileTrue(new EncoderandArm(bar, arm, 38500)); // 8
   
    lockSolenoid.onTrue(new SolenoidCommand(arm)); // 7


    
    turretRight.whileTrue(new TurretTestCommand(turret, 0.5));
    turretLeft.whileTrue(new TurretTestCommand(turret, -0.5));
    rightBumper.whileTrue(new ClawToggle(claw));  
    leftTrigger.whileTrue(new VirtualFourBarCommand(bar, arm, 0.3));  
    rightTrigger.whileTrue(new VirtualFourBarCommand(bar, arm, -0.3));  


    /*Actual Command Mapping */






    /*    TestButton Mapping */
    // JoystickButton clawToggle = new JoystickButton(buttonPanel, 1);
    // JoystickButton armToggle = new JoystickButton(buttonPanel, 2);
    // JoystickButton clawOut = new JoystickButton(buttonPanel, 3);
    // JoystickButton clawIn = new JoystickButton(buttonPanel, 4);
    // JoystickButton vfBarD = new JoystickButton(buttonPanel, 5);
    // JoystickButton vfBarUp = new JoystickButton(buttonPanel, 6);
    // JoystickButton lockSolenoid = new JoystickButton(buttonPanel, 7);
    // JoystickButton turretL = new JoystickButton(buttonPanel, ButtonConstants.turretL);
    // JoystickButton turretR = new JoystickButton(buttonPanel, 9);
    // JoystickButton midCone = new JoystickButton(buttonPanel, 10);
    /* TestButton Mapping */


    /* Test Mapping */
    // clawToggle.onTrue(new ClawToggle(claw)); //1
    // armToggle.onTrue(new ArmToggleCommand(arm)); //2
    // clawOut.whileTrue(new ClawOuttake(claw, -1)); //3
    // clawIn.whileTrue(new ClawIntake(claw, 1)); //4
    // vfBarD.whileTrue(new VirtualFourBarCommand(bar, arm, 0.3)); //5
    // vfBarUp.whileTrue(new VirtualFourBarCommand(bar, arm, -0.3)); //6
    // lockSolenoid.onTrue(new SolenoidCommand(arm)); //7
    // turretL.whileTrue(new TurretTestCommand(turret, 0.5)); //8
    // turretR.whileTrue(new TurretTestCommand(turret, -0.5)); //9
    // midCone.whileTrue(new EncoderandArm(bar, arm, 62464)); //10
    /* Test Mapping */

    /* Command Mapping */

  }

  public void robotPeriodic() {
    while(color.getProximity()>1000){
      claw.extend();
    }
  }
  

  public Command getAutonomousCommand() {
    return chooser.getSelected();
}
}
