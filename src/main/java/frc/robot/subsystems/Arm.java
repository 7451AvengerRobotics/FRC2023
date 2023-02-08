package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.PortConstants;

public class Arm {
    public final CANSparkMax arm;
    public final SparkMaxPIDController pidController;
    public final RelativeEncoder encoder;
    private static final double ARM_GEAR_RATIO = 1;
    private static final double ARM_HIGH = 200;
    private static final double ARM_LOW = 100;

    public Arm() {
        arm = new CANSparkMax(PortConstants.Arm, MotorType.kBrushless);
        pidController = arm.getPIDController();
        encoder = arm.getEncoder();
        arm.restoreFactoryDefaults();
        pidController.setOutputRange(-1, 1);
        // armTurnMotor.setClosedLoopRampRate(.5);
        arm.enableSoftLimit(SoftLimitDirection.kForward, true);
        arm.enableSoftLimit(SoftLimitDirection.kReverse, true);
        arm.setIdleMode(IdleMode.kBrake);
        arm.setSoftLimit(SoftLimitDirection.kForward, 24f);
        arm.setSoftLimit(SoftLimitDirection.kReverse, -24f);
        encoder.setPosition(0);
    }


    public void presetTo(double counts) {
        if (counts == 0) {
            pidController.setReference(0, CANSparkMax.ControlType.kPosition);
        } else {
            pidController.setReference(counts * ARM_GEAR_RATIO, CANSparkMax.ControlType.kPosition);
        }
    }   
}
