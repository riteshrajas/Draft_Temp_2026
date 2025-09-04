package frc.robot.robotActions;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.utils.DrivetrainConstants;
import frc.robot.utils.advancedcontrollers.Action;


public class Swerve {
    // Swerve drive actions with configurable speeds

    public static Action<Double> driveForward = Action.set(
        (Double speed) -> {
            SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(speed)
                .withVelocityY(0.0)
                .withRotationalRate(0);
            DrivetrainConstants.drivetrain.setControl(request);
        },
        1.0
    );

    public static Action<Double> driveBackward = Action.set(
        (Double speed) -> {
            SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(-speed)
                .withVelocityY(0.0)
                .withRotationalRate(0);
            DrivetrainConstants.drivetrain.setControl(request);
        },
        1.0
    );

    public static Action<Double> strafeLeft = Action.set(
        (Double speed) -> {
            SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(0.0)
                .withVelocityY(-speed)
                .withRotationalRate(0);
            DrivetrainConstants.drivetrain.setControl(request);
        },
        1.0
    );

    public static Action<Double> strafeRight = Action.set(
        (Double speed) -> {
            SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(0.0)
                .withVelocityY(speed)
                .withRotationalRate(0);
            DrivetrainConstants.drivetrain.setControl(request);
        },
        1.0
    );

    // Stop all movement
    public static Action<Double> stop = Action.set(
        (Double unused) -> {
            SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0);
            DrivetrainConstants.drivetrain.setControl(request);
        },
        0.0
    );

}