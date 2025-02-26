package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase {

    private final NetworkTable limelightTable;
    private double[] targetpose = NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("targetpose_robotspace")
            .getDoubleArray(new double[6]);

    // These parameters should be tuned to match your robot and target setup.
    private final double targetHeight = 2.5;   // Height of the target in meters (example)
    private final double cameraHeight = 1.0;   // Height of the Limelight in meters (example)
    private final double cameraAngle = 15.0;   // Angle of the Limelight in degrees (example)

    public VisionSubsystem() {
        // Connect to the limelight NetworkTable
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        targetpose = NetworkTableInstance.getDefault()
                .getTable("limelight")
                .getEntry("targetpose_robotspace")
                .getDoubleArray(new double[0]);
    }

    /**
     * Returns true if the Limelight has a valid target.
     */
    public boolean isTargetVisible() {
        // "tv" is 1.0 when a target is visible, 0.0 otherwise.
        boolean visible = limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
        SmartDashboard.putBoolean("Target Visible", visible);
        return visible;
    }

    /**
     * Returns the horizontal offset (in degrees) from the crosshair to the target.
     */
    public double getTargetAngle() {
        // "tx" gives the horizontal offset from crosshair to target.
        double angle = limelightTable.getEntry("tx").getDouble(0.0);
        SmartDashboard.putNumber("Target Angle", angle);
        return angle;
    }

    /**
     * Computes the distance from the robot to the target using the vertical angle offset.
     * Uses the formula: distance = (targetHeight - cameraHeight) / tan(cameraAngle + ty)
     */
    public double getTargetDistance() {
        // "ty" gives the vertical offset from crosshair to target.
        double ty = limelightTable.getEntry("ty").getDouble(0.0);
        double angleToTargetRadians = Math.toRadians(targetpose[4] + ty);

        // Prevent division by zero.
        if (Math.tan(angleToTargetRadians) == 0) {
            return 0;
        }

        double distance = targetpose[0];
        SmartDashboard.putNumber("Target Distance", distance);
        return distance;
    }
}