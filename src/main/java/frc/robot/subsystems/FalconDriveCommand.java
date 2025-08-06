package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class FalconDriveCommand extends Command {
    private static final double STICK_DEADBAND = 0.1;

    private final CommandSwerveDrivetrain swerve;
    private final XboxController controller;
    private final double maxSpeed;
    private final double maxAngularRate;
    private boolean isFieldCentric = true;

    public FalconDriveCommand(CommandSwerveDrivetrain swerve, XboxController controller, double maxSpeed) {
        this.swerve = swerve;
        this.controller = controller;
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = Math.PI * 2; // 1 rotation per second
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Field Centric", isFieldCentric);

        // Get stick values with deadband
        double leftX = applyDeadband(-controller.getLeftY(), STICK_DEADBAND); // Forward/backward
        double leftY = applyDeadband(-controller.getLeftX(), STICK_DEADBAND); // Left/right
        double rightX = applyDeadband(-controller.getRightX(), STICK_DEADBAND); // Rotation

        // Scale to max speeds
        double velocityX = leftX * maxSpeed;
        double velocityY = leftY * maxSpeed;
        double angularRate = rightX * maxAngularRate;

        SmartDashboard.putNumber("Velocity X", velocityX);
        SmartDashboard.putNumber("Velocity Y", velocityY);
        SmartDashboard.putNumber("Angular Rate", angularRate);

        if (isFieldCentric) {
            // Field-centric mode
            var fieldCentricRequest = new FieldCentric()
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(angularRate)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

            swerve.setControl(fieldCentricRequest);
        } else {
            // Robot-centric mode
            var robotCentricRequest = new RobotCentric()
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(angularRate)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

            swerve.setControl(robotCentricRequest);
        }
    }

    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return (value - Math.copySign(deadband, value)) / (1.0 - deadband);
    }

    public void toggleDriveMode() {
        isFieldCentric = !isFieldCentric;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
