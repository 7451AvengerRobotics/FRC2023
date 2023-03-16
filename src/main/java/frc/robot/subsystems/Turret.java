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
        

//Configures the PID Slot to 0 as well as sets the soft limits to ensure the turrret does not move 180
        turret.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, TurretConstants.kTimeoutMs);
        turret.configForwardSoftLimitThreshold(680000);
        turret.configReverseSoftLimitThreshold(-642073);
        turret.configForwardSoftLimitEnable(true);
        turret.configReverseSoftLimitEnable(true);

//Setting the PID Values
        turret.config_kP(0, TurretConstants.kGains.kP, TurretConstants.kTimeoutMs);
		turret.config_kI(0, TurretConstants.kGains.kI, TurretConstants.kTimeoutMs);
		turret.config_kD(0, TurretConstants.kGains.kD, TurretConstants.kTimeoutMs);

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
//Will Change the turret position in conjunction with Limelight
    public void turretAdjust(double counts){
        turret.set(TalonFXControlMode.Position, 100, DemandType.AuxPID, counts);
        turret.set(TalonFXControlMode.Position, counts);
    }
}
