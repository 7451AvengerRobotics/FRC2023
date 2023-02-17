package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase{
    public final TalonFX turret;
    public final TalonFXConfiguration turretConfig;
    boolean turretState;
    boolean firstCall;
    public Turret() {
        turret = new TalonFX(PortConstants.Turret);
        turret.setNeutralMode(NeutralMode.Coast);
        turret.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        turret.configForwardSoftLimitThreshold(10000);//Values Subject To Change
        turret.configReverseSoftLimitThreshold(12000);//Values subject to change
        turret.configForwardSoftLimitEnable(true);
        turret.configReverseSoftLimitEnable(true);
        turretConfig = new TalonFXConfiguration();
        firstCall = false;
        turretState = false;

        turretConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        turretConfig.neutralDeadband = 0.001;
        turretConfig.peakOutputForward = 0.7;
        turretConfig.peakOutputReverse = -0.7;

        turretConfig.slot0.kP = TurretConstants.kGains_Distanc.kP;
		turretConfig.slot0.kI = TurretConstants.kGains_Distanc.kI;
		turretConfig.slot0.kD = TurretConstants.kGains_Distanc.kD;
		turretConfig.slot0.kF = TurretConstants.kGains_Distanc.kF;
		turretConfig.slot0.integralZone = TurretConstants.kGains_Distanc.kIzone;
		turretConfig.slot0.closedLoopPeakOutput = TurretConstants.kGains_Distanc.kPeakOutput;
		turretConfig.slot0.allowableClosedloopError = 0;

        int closedLoopTimeMs = 1;
		turretConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
		turretConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
		turretConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
		turretConfig.slot3.closedLoopPeriod = closedLoopTimeMs;

        turret.configAllSettings(turretConfig);

        turret.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, TurretConstants.kTimeoutMs);
		turret.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, TurretConstants.kTimeoutMs);
		turret.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, TurretConstants.kTimeoutMs);
		turret.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, TurretConstants.kTimeoutMs);
		
		/* Initialize */
		firstCall = true;
		turretState = false;
		zeroSensors();




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

public void zeroSensors() {
		turret.getSensorCollection().setIntegratedSensorPosition(0, TurretConstants.kTimeoutMs);
	}
}
