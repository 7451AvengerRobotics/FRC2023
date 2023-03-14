package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.TurretConstants;

public class VirtualFourBar extends SubsystemBase {

    private final WPI_TalonFX vFBAR;

    public VirtualFourBar(){
        vFBAR = new WPI_TalonFX(PortConstants.VirtualFourBar);
        vFBAR.setNeutralMode(NeutralMode.Brake);
        vFBAR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        
        vFBAR.setSensorPhase(true);
        vFBAR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);  
        vFBAR.config_kP(0, getencoderValues());
        vFBAR.setSelectedSensorPosition(getencoderValues());      

        vFBAR.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, TurretConstants.kTimeoutMs);
        vFBAR.configForwardSoftLimitThreshold(98590);
        vFBAR.configReverseSoftLimitThreshold(-300);//Values subject to change
        vFBAR.configForwardSoftLimitEnable(true);
        vFBAR.configReverseSoftLimitEnable(true);
        //Directly forward is 46511


    }


    public void setPower(double power){
        vFBAR.set(ControlMode.PercentOutput, power);
    }

    public void setPosition(double encoderPosition){
        vFBAR.set(ControlMode.Position, encoderPosition);
    }

    
    public double getencoderValues() {
        return vFBAR.getSelectedSensorPosition();
    }

    public void zeroSensors() {
        vFBAR.getSensorCollection().setIntegratedSensorPosition(0, TurretConstants.kTimeoutMs);
    }

}
