package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.PortConstants;

public class AutoBalance {
    public static boolean run() {
        Pigeon2 gyro = new Pigeon2(PortConstants.Gyro);
        double speed = 0.5;
        double rollAngleDegrees = gyro.getRoll();
        Drivetrain drivetrain = new Drivetrain();
        while (!(!Drivetrain.autobalanceXMode && (Math.abs(rollAngleDegrees) >= Math.abs(Drivetrain.kOffBalanceAngleThresholdDegrees)))) {
            drivetrain.setPower(speed);
        }
        double yawAngleRadians = rollAngleDegrees * (Math.PI / 180);
        speed = Math.sin(yawAngleRadians) * -0.5;
        try {
            drivetrain.setPower(speed);
        } catch (RuntimeException ex) {
            String err_string = "Drive system error " + ex.getMessage();
            DriverStation.reportError(err_string, true);
        }

        System.out.println("speed: " + speed + "yaw: " + rollAngleDegrees);
        return true;
    }
}
