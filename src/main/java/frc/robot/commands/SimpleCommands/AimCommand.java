package frc.robot.commands.SimpleCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Limelight;



public class AimCommand extends CommandBase {

    private final Turret turret;
    private final Limelight limelight;

    public AimCommand(Turret turret, Limelight limelight){
        this.turret = turret;
        this.limelight = limelight;
        addRequirements(turret, limelight);

    }

    @Override
    public void initialize() {
      limelight.enableLights();
    }
  
    @Override
    public void execute() {
      turret.turnWithEncoders(limelight.getRotationAdjust());
    }
  
    @Override
    public void end(boolean interrupted) {
      System.out.println("Limelight OFF");
      limelight.disableLights();
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
    
}
