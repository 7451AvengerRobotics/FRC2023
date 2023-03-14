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
        turret = new TalonFX(PortConstants.Turret);
        turret.setNeutralMode(NeutralMode.Brake);

        turret.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        

        turret.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, TurretConstants.kTimeoutMs);
        turret.configForwardSoftLimitThreshold(680000);
        turret.configReverseSoftLimitThreshold(-642073);//Values subject to change
        turret.configForwardSoftLimitEnable(true);
        turret.configReverseSoftLimitEnable(true);


        // turret.configNominalOutputForward(0, TurretConstants.kTimeoutMs);
		// turret.configNominalOutputReverse(0, TurretConstants.kTimeoutMs);
		// turret.configPeakOutputForward(1, TurretConstants.kTimeoutMs);
		// turret.configPeakOutputReverse(-1, TurretConstants.kTimeoutMs);

        turret.configAllowableClosedloopError(0, 
        0, 
        TurretConstants.kTimeoutMs);

        turret.config_kF(0, TurretConstants.kGains.kF, TurretConstants.kTimeoutMs);
		turret.config_kP(0, TurretConstants.kGains.kP, TurretConstants.kTimeoutMs);
		turret.config_kI(0, TurretConstants.kGains.kI, TurretConstants.kTimeoutMs);
		turret.config_kD(0, TurretConstants.kGains.kD, TurretConstants.kTimeoutMs);

    }

    public void turn(double power) {
        turret.set(ControlMode.PercentOutput, power);
    }

    public double getencoderValues() {
        return turret.getSelectedSensorPosition();
    }

    public void turnWithEncoders(double counts) {
        turret.set(TalonFXControlMode.Position, counts);
    }

    public void zeroSensors() {
        turret.getSensorCollection().setIntegratedSensorPosition(0, TurretConstants.kTimeoutMs);
    }

    public void turretAdjust(double counts){
        turret.set(TalonFXControlMode.Position, 100, DemandType.AuxPID, counts);
        turret.set(TalonFXControlMode.Position, counts);
    }
}
