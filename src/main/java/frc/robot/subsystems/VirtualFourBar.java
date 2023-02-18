package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants.PortConstants;

public class VirtualFourBar {

    private final WPI_TalonFX vFBAR;

    public VirtualFourBar(){
        vFBAR = new WPI_TalonFX(PortConstants.VirtualFourBar);
    }



    public void setPower(double power){
        vFBAR.set(power);
    }

}
