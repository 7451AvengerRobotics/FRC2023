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
        turret.setNeutralMode(NeutralMode.Coast);
        turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        turret.configForwardSoftLimitThreshold(10000);//Values Subject To Change
        turret.configReverseSoftLimitThreshold(10000);//Values subject to change
        turret.configForwardSoftLimitEnable(true);
        turret.configReverseSoftLimitEnable(true);


    }
public void turn(double power) {
    turret.set(ControlMode.PercentOutput, power);
}   

public double getencoderValues(){
    return turret.getSelectedSensorPosition();
}

public void turnWithEncoders(double counts) {
      turret.setSelectedSensorPosition(counts);
    }
}
