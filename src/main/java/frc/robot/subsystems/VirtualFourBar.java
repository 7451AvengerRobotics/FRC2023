package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class VirtualFourBar extends SubsystemBase {

    private final WPI_TalonFX vFBAR;

    public VirtualFourBar(){
        vFBAR = new WPI_TalonFX(PortConstants.VirtualFourBar);
        vFBAR.setNeutralMode(NeutralMode.Brake);
        vFBAR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
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

}
