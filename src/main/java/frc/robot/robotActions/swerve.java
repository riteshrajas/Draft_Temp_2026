package frc.robot.robotActions;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.utils.DrivetrainConstants;
import frc.robot.utils.advancedcontrollers.Action;


public class swerve {
    // Create a reusable drive request for forward movement
    private static final SwerveRequest.RobotCentric forwardRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(1.0) // 1 m/s forward
            .withVelocityY(0.0)
            .withRotationalRate(0);

    private static final SwerveRequest.RobotCentric backwardRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(-1.0) // 1 m/s backward
            .withVelocityY(0.0)
            .withRotationalRate(0);

    private static final SwerveRequest.RobotCentric leftRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(0.0)
            .withVelocityY(-1.0) // 1 m/s left
            .withRotationalRate(0);

    private static final SwerveRequest.RobotCentric rightRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(0.0)
            .withVelocityY(1.0) // 1 m/s right
            .withRotationalRate(0);

    public static Action<Void> driveForward = new Action<>(
        (ritesh) -> DrivetrainConstants.drivetrain.setControl(forwardRequest),
        null
    );

    public static Action<Void> driveBackward = new Action<>(
        (ritesh) -> DrivetrainConstants.drivetrain.setControl(backwardRequest),
        null
    );

    public static Action<Void> strafeLeft = new Action<>(
        (ritesh) -> DrivetrainConstants.drivetrain.setControl(leftRequest),
        null
    );

    public static Action<Void> strafeRight = new Action<>(
        (ritesh) -> DrivetrainConstants.drivetrain.setControl(rightRequest),
        null
    );

    // public static Action<Void> reset = new Action<>(
    //     (ritesh) -> {
    //         drivetrain.applyRequest(() -> {
    //         double kAngleP = 5;
    //         double maxAngularRate = Math.PI;

    //         Rotation2d currentRotation = drivetrain.getState().Pose.getRotation();
    //         Rotation2d targetRotation = DrivetrainConstants.startingDirection;

    //         double errorRad = targetRotation.minus(currentRotation).getRadians();

    //         double rotationalRateRadPerSec = errorRad * kAngleP;

    //         rotationalRateRadPerSec = Math.max(-maxAngularRate, Math.min(maxAngularRate, rotationalRateRadPerSec));
    //         if (stayOnTarget) {
    //             DrivetrainConstants.lastTargetDirection = targetRotation;
    //         }
    //         DrivetrainConstants.lastTargetDirection = targetRotation;
    //         return new SwerveRequest.FieldCentric()
    //                 .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    //                 .withVelocityX(0)
    //                 .withVelocityY(0)
    //                 .withRotationalRate(rotationalRateRadPerSec);
    //     });


}