package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase{
    public final TalonFX turret;
    boolean turretState;
    boolean firstCall;
    public Turret() {
        turret = new TalonFX(PortConstants.Turret);
        turret.setNeutralMode(NeutralMode.Brake);






    }
public void turn(double power) {
    turret.set(ControlMode.PercentOutput, power);
}   

public double getencoderValues(){
    return turret.getSelectedSensorPosition();
}



public void turnWithEncoders(double counts) {
    turret.set(TalonFXControlMode.Position, counts);
}

public void zeroSensors() {
		turret.getSensorCollection().setIntegratedSensorPosition(0, TurretConstants.kTimeoutMs);
	}
}
