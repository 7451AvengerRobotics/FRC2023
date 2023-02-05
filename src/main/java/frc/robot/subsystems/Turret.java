package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.PortConstants;

public class Turret {
    public final CANSparkMax turret;
    public final SparkMaxPIDController pidController;
    public final RelativeEncoder encoder;
    private static final double TURRET_GEAR_RATIO = 15;
    public Turret() {
        turret = new CANSparkMax(PortConstants.Turret, MotorType.kBrushless);
        pidController = turret.getPIDController();
        encoder = turret.getEncoder();
        turret.restoreFactoryDefaults();
        pidController.setOutputRange(-0.8, 0.8);
        // turretTurnMotor.setClosedLoopRampRate(.5);
        turret.enableSoftLimit(SoftLimitDirection.kForward, true);
        turret.enableSoftLimit(SoftLimitDirection.kReverse, true);
        turret.setIdleMode(IdleMode.kBrake);
        turret.setSoftLimit(SoftLimitDirection.kForward, 24f);
        turret.setSoftLimit(SoftLimitDirection.kReverse, -24f);
        encoder.setPosition(0);
    }
public void turn(double counts) {
    if (counts == 0) {
        pidController.setReference(0, CANSparkMax.ControlType.kPosition);
    } else {
        pidController.setReference(counts * TURRET_GEAR_RATIO, CANSparkMax.ControlType.kPosition);
    }
}   
}
