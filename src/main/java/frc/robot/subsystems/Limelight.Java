package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{
        public static NetworkTable table;
        // how many degrees back is your limelight rotated from perfectly vertical?
        private static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 35.00;
        // distance from the center of the Limelight lens to the floor
        private static final double LIMELIGHT_LENS_HEIGHT_METERS = 0.79;
        // distance from the target to the floor
        private static final double GOAL_HEIGHT_METERS = 2.6416;
    
        private static final double Kp = 0.02; // Proportional control constant
        private static final double min_command = 0.15; // Minimum amount to slightly move
        private double steering_adjust;
    
        public Limelight() {
            table = NetworkTableInstance.getDefault().getTable("limelight-b");
    
            table.getEntry("pipeline").setNumber(0);
          }
    
        @Override
        public void periodic() {
        System.out.println(getDistance());
      }

      public double getRotationAdjust() {
        double tx = table.getEntry("tx").getDouble(0);
        double heading_error = -tx;
        if (tx > 1.0) {
          steering_adjust = Kp * heading_error - min_command;
        } else if (tx < 1.0) {
          steering_adjust = Kp * heading_error + min_command;
        }
        return steering_adjust;
      }

    public static boolean getDistance() {
        double targetOffsetAngle_Vertical = table.getEntry("ty").getDouble(0);

        double angle = LIMELIGHT_MOUNT_ANGLE_DEGREES + targetOffsetAngle_Vertical;
        double height = GOAL_HEIGHT_METERS - LIMELIGHT_LENS_HEIGHT_METERS;

        // tan = opp / hypot; divide opp to get adj.
        double distance = height / Math.tan(Math.toRadians(angle));
        return SmartDashboard.putNumber("Distance To Hub", Units.metersToInches(distance));
    }

    public static double getDistanceDouble() {
        double targetOffsetAngle_Vertical = table.getEntry("ty").getDouble(0);

        double angle = LIMELIGHT_MOUNT_ANGLE_DEGREES + targetOffsetAngle_Vertical;
        double height = GOAL_HEIGHT_METERS - LIMELIGHT_LENS_HEIGHT_METERS;

        // tan = opp / hypot; divide opp to get adj.
        double distance = height / Math.tan(Math.toRadians(angle));
        return Units.metersToInches(distance);
    }

    public void disableLights() {
        // Disables LEDs
        table.getEntry("ledMode").setNumber(0);
      }

      public void enableLights() {
        // Enables LEDs
        table.getEntry("ledMode").setNumber(3);
      }
    
    }
