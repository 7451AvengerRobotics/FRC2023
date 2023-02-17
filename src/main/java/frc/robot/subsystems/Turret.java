package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants.PortConstants;

public class Turret {
    public final TalonFX turret;
    public Turret() {
        turret = new TalonFX(PortConstants.Turret);
        turret.setNeutralMode(NeutralMode.Brake);
        turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    }
public void turn(double power) {
    turret.set(ControlMode.PercentOutput, power);
}   

public double getencoderValues(){
    return turret.getSelectedSensorPosition();
}
}
