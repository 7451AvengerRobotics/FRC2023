package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
    public final TalonFX turret;
    boolean turretState;
    boolean firstCall;

    public Turret() {
//Creates the Turret Class
        turret = new TalonFX(PortConstants.Turret);
        turret.setNeutralMode(NeutralMode.Brake);
        
//Configures the sensor to an integrated sensor
        turret.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        

        turret.configForwardSoftLimitThreshold(680000);
        turret.configReverseSoftLimitThreshold(-642073);
        turret.configForwardSoftLimitEnable(true);
        turret.configReverseSoftLimitEnable(true);


    }

//Setting Power with percent output
    public void turn(double power) {
        turret.set(ControlMode.PercentOutput, power);
    }

//Getting the Encoder position of the turret
    public double getencoderValues() {
        return turret.getSelectedSensorPosition();
    }

//Setting Power with position output
    public void turnWithEncoders(double counts) {
        turret.set(TalonFXControlMode.Position, counts);
    }

//Setting the Encoder position of the turret to 0
    public void zeroSensors() {
        turret.getSensorCollection().setIntegratedSensorPosition(0, TurretConstants.kTimeoutMs);
    }
}
