package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// made by justin, command for auto driving and rotating toward an AprilTag using Limelight until the tag is lost
public class AprilTag extends Command {
    private final CANDriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final double targetDistance;
    private final PIDController pidForward;
    private final PIDController pidTurn;
    private double forwardSpeed;
    private double rotationSpeed;

    public AprilTag(CANDriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double targetDistance) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.targetDistance = targetDistance;
        // PID controller for driving forward; tune these constants as needed
        this.pidForward = new PIDController(0.0025, 0, 0);
        // PID controller for turning; tune these constants as needed (e.g., using degrees error)
        this.pidTurn = new PIDController(0.02, 0, 0);
        addRequirements(driveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        pidForward.reset();
        pidTurn.reset();
        SmartDashboard.putNumber("TargetDistance", targetDistance);
    }

    @Override
    public void execute() {
        // Use Limelight data via the VisionSubsystem to check for an AprilTag and its properties
        if (visionSubsystem.isTargetVisible()) {
            // Get current distance from the target as computed by the Limelight
            double currentDistance = visionSubsystem.getTargetDistance();
            // Calculate the forward PID output to drive toward the target distance
            forwardSpeed = pidForward.calculate(currentDistance, targetDistance);

            // Get the horizontal offset (angle) from the crosshair to the target (in degrees)
            double targetAngle = visionSubsystem.getTargetAngle();
            // Calculate the turning correction (aiming to reduce the angle to 0)
            rotationSpeed = pidTurn.calculate(targetAngle, 0);

            // Combine forward and rotation speeds for differential drive
            double leftSpeed = forwardSpeed + rotationSpeed;
            double rightSpeed = forwardSpeed - rotationSpeed;
            driveSubsystem.setSpeed(leftSpeed, rightSpeed);

            // Output values to SmartDashboard for tuning/monitoring
            SmartDashboard.putNumber("ForwardPIDOutput", forwardSpeed);
            SmartDashboard.putNumber("RotationPIDOutput", rotationSpeed);
            SmartDashboard.putNumber("CurrentTagDistance", currentDistance);
            SmartDashboard.putNumber("TargetAngle", targetAngle);
        } else {
            // If the target is lost, stop the robot
            driveSubsystem.setSpeed(0, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends or is interrupted
        driveSubsystem.setSpeed(0, 0);
    }

    @Override
    public boolean isFinished() {
        // Finish the command when the Limelight no longer sees the AprilTag
        return !visionSubsystem.isTargetVisible();
    }
}