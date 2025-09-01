package frc.robot.framework;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.DrivetrainConstants;
import frc.robot.utils.preferences.RobotPreferences;

/**
 * Class for configuring robot drive modes.
 */
public class RobotFramework {

    protected final CommandSwerveDrivetrain drivetrain = DrivetrainConstants.drivetrain;

    protected final CommandXboxController joystick = new CommandXboxController(0);
    // Stores the joystick's angle from the previous loop cycle to calculate its speed
    private double previousJoystickAngle = 0.0; 
    // A timestamp from the last time the angle was updated, for a more accurate velocity calculation
    private double lastAngleTimestamp = 0.0;
    // protected final CommandXboxController joystick = new CommandXboxController(0);

    protected final TeleOpBuilder TeleOpBuilder = new TeleOpBuilder(drivetrain, joystick, DrivetrainConstants.prefs);


    public class TeleOpBuilder {
        private final CommandSwerveDrivetrain drivetrain;
        private final CommandXboxController controller;
        private final RobotPreferences prefs;

        public TeleOpBuilder(CommandSwerveDrivetrain drivetrain, CommandXboxController controller, RobotPreferences prefs) {
            this.drivetrain = drivetrain;
            this.controller = controller;
            this.prefs = prefs;
        }

        public SendableChooser<Command> buildTeleOpChooser(String defaultMode) {
            SendableChooser<Command> chooser = new SendableChooser<>();
            chooser.setDefaultOption(defaultMode, buildHolonomicDrive(DrivetrainConstants.maxSpeed, DrivetrainConstants.maxAngularSpeed));
            chooser.addOption("Arcade Drive", buildArcadeDrive(DrivetrainConstants.maxSpeed, DrivetrainConstants.maxAngularSpeed));
            chooser.addOption("Tank Drive", buildTankDrive(DrivetrainConstants.maxSpeed, DrivetrainConstants.maxAngularSpeed));
            chooser.addOption("Falcon Drive", buildFalconDrive(DrivetrainConstants.maxSpeed, DrivetrainConstants.maxAngularSpeed));
            return chooser;
        }

        public Command buildHolonomicDrive(double maxSpeed, double maxAngular) {
            return applyHolonomicDrive(drivetrain, controller, prefs, maxSpeed, maxAngular);
        }

        public Command buildArcadeDrive(double maxSpeed, double maxAngular) {
            return applyArcadeDrive(drivetrain, controller, prefs, maxSpeed, maxAngular);
        }

        public Command buildTankDrive(double maxSpeed, double maxAngular) {
            return applyTankDrive(drivetrain, controller, prefs, maxSpeed, maxAngular);
        }

        public Command buildFalconDrive(double maxSpeed, double maxAngular) {
            return applyFalconDrive(drivetrain, controller, prefs, maxSpeed, maxAngular);
        }
    }

    public RobotFramework() {
        // Initialize preferences

    }

    public Command applyHolonomicDrive(CommandSwerveDrivetrain drivetrain, CommandXboxController controller, RobotPreferences prefs, double maxSpeed, double maxAngular) {
        if (prefs.isFieldCentricEnabled())
            return drivetrain.applyRequest(() -> DrivetrainConstants.fieldCentric
                .withVelocityX(DrivetrainConstants.xLimiter.calculate(-controller.getLeftY() * maxSpeed * prefs.getRobotSpeedMultiplier()))
                .withVelocityY(DrivetrainConstants.yLimiter.calculate(-controller.getLeftX() * maxSpeed * prefs.getRobotSpeedMultiplier()))
                .withRotationalRate((-controller.getRightX() * maxAngular * prefs.getRotationSpeedMultiplier())));
        else
            return drivetrain.applyRequest(() -> DrivetrainConstants.robotCentric
                .withVelocityX(DrivetrainConstants.xLimiter.calculate(-controller.getLeftY() * maxSpeed * prefs.getRobotSpeedMultiplier()))
                .withVelocityY(DrivetrainConstants.yLimiter.calculate(-controller.getLeftX() * maxSpeed * prefs.getRobotSpeedMultiplier()))
                .withRotationalRate((-controller.getRightX() * maxAngular * prefs.getRotationSpeedMultiplier())));
    }

    public Command applyArcadeDrive(CommandSwerveDrivetrain drivetrain, CommandXboxController controller, RobotPreferences prefs, double maxSpeed, double maxAngular) {
        if (prefs.isFieldCentricEnabled())
            return drivetrain.applyRequest(() -> DrivetrainConstants.fieldCentric
                .withVelocityX(DrivetrainConstants.xLimiter.calculate(-controller.getLeftY() * maxSpeed * prefs.getRobotSpeedMultiplier()))
                .withRotationalRate((-controller.getRightX() * maxAngular * prefs.getRotationSpeedMultiplier())));
        else
            return drivetrain.applyRequest(() -> DrivetrainConstants.robotCentric
                .withVelocityX(DrivetrainConstants.xLimiter.calculate(-controller.getLeftY() * maxSpeed * prefs.getRobotSpeedMultiplier()))
                .withRotationalRate((-controller.getRightX() * maxAngular * prefs.getRotationSpeedMultiplier())));
    }

    public Command applyTankDrive(CommandSwerveDrivetrain drivetrain, CommandXboxController controller, RobotPreferences prefs, double maxSpeed, double maxAngular) {
        if (prefs.isFieldCentricEnabled())
            return drivetrain.applyRequest(() -> {
                double leftInput = -controller.getLeftY();
                double rightInput = -controller.getRightY();
                double velocityX = (leftInput + rightInput) / 2.0 * maxSpeed * prefs.getRobotSpeedMultiplier();
                double rotationRate = (rightInput - leftInput) / 2.0 * maxAngular * prefs.getRotationSpeedMultiplier();
                return DrivetrainConstants.fieldCentric
                        .withVelocityX(velocityX)
                        .withRotationalRate(rotationRate);
            });
        else
        return drivetrain.applyRequest(() -> {
            double leftInput = -controller.getLeftY();
            double rightInput = -controller.getRightY();
            double velocityX = (leftInput + rightInput) / 2.0 * maxSpeed * prefs.getRobotSpeedMultiplier();
            double rotationRate = (rightInput - leftInput) / 2.0 * maxAngular * prefs.getRotationSpeedMultiplier();
            return DrivetrainConstants.robotCentric
                    .withVelocityX(velocityX)
                    .withRotationalRate(rotationRate);
        });
    }

public Command applyFalconDrive(CommandSwerveDrivetrain drivetrain, CommandXboxController controller, RobotPreferences prefs, double maxSpeed, double maxAngular) {
    
    return drivetrain.applyRequest(() -> {
        double rightX = -joystick.getRightX();
        double rightY = -joystick.getRightY();

        if (Math.hypot(rightX, rightY) > 0.7) { // Deadband check
            // --- PREDICTION LOGIC START ---

            // 1. Calculate current angle and normalize to [0, 360)
            double currentJoystickAngle = (Math.toDegrees(Math.atan2(-rightY, -rightX)) + 90);
            currentJoystickAngle = (currentJoystickAngle % 360 + 360) % 360;

            // 2. Calculate joystick's angular velocity
            double currentTime = Timer.getFPGATimestamp();
            double deltaTime = currentTime - lastAngleTimestamp;
            
            // To avoid division by zero on the first run
            if (deltaTime == 0) {
                deltaTime = 0.02; // Assume a 50Hz loop rate initially
            }

            // Find the shortest angle difference to handle the 359 -> 0 degree wrap-around
            double angleDifference = currentJoystickAngle - previousJoystickAngle;
            if (angleDifference > 180) angleDifference -= 360;
            if (angleDifference < -180) angleDifference += 360;

            double joystickAngularVelocity = angleDifference / deltaTime; // degrees per second

            // 3. Calculate the "lead" angle based on velocity
            // This is the amount we'll predict ahead. The faster the stick moves, the more we lead.
            // The '0.05' is a tuning factor. Increase it for more prediction, decrease for less.
            double leadAngle = joystickAngularVelocity * 0.05;

            // Clamp the lead angle to a reasonable maximum (e.g., 20 degrees) to prevent instability
            leadAngle = Math.max(-20.0, Math.min(20.0, leadAngle));

            // 4. Determine the base snapped angle
            double baseSnappedAngle = (int)(currentJoystickAngle / 15.0) * 15.0 ;

            // 5. Apply the prediction
            // The final target is the snapped angle plus our predictive lead
            double finalTargetAngle = baseSnappedAngle + leadAngle;

            // --- PREDICTION LOGIC END ---

            // Set the target direction
            DrivetrainConstants.lastTargetDirection = Rotation2d.fromDegrees(finalTargetAngle);
            SmartDashboard.putNumber("Target Angle", finalTargetAngle);
            SmartDashboard.putNumber("Joystick Velocity (dps)", joystickAngularVelocity);

            // Update state for the next loop
            previousJoystickAngle = currentJoystickAngle;
            lastAngleTimestamp = currentTime;
        }
        
        Rotation2d currentRotation = drivetrain.getState().Pose.getRotation();
        double errorRad = DrivetrainConstants.lastTargetDirection.minus(currentRotation).getRadians();
        double rotationalRateRadPerSec = errorRad * 5;

        rotationalRateRadPerSec = Math.max(-DrivetrainConstants.getMaxAngularSpeed(), Math.min(DrivetrainConstants.getMaxAngularSpeed(), rotationalRateRadPerSec));

        if (prefs.isFieldCentricEnabled())
            return DrivetrainConstants.fieldCentric
                    .withVelocityX(DrivetrainConstants.xLimiter.calculate(controller.getLeftY() * maxSpeed * prefs.getRobotSpeedMultiplier()))
                    .withVelocityY(DrivetrainConstants.yLimiter.calculate(controller.getLeftX() * maxSpeed * prefs.getRobotSpeedMultiplier()))
                    .withRotationalRate(rotationalRateRadPerSec);
        else
            return DrivetrainConstants.robotCentric
                    .withVelocityX(DrivetrainConstants.xLimiter.calculate(controller.getLeftY() * maxSpeed * prefs.getRobotSpeedMultiplier()))
                    .withVelocityY(DrivetrainConstants.yLimiter.calculate(controller.getLeftX() * maxSpeed * prefs.getRobotSpeedMultiplier()))
                    .withRotationalRate(rotationalRateRadPerSec);
    }); 
}



    public Command getRotationDegreesCommand(double degrees) {
        /* Create a command to rotate the robot to a specific angle with a P-controller */
        return drivetrain.applyRequest(() -> {
            double kAngleP = 5; // Proportional gain for angle controller - TUNE THIS
            double maxAngularRate = Math.PI; // Max rotational speed in rad/s - TUNE THIS

            Rotation2d currentRotation = drivetrain.getState().Pose.getRotation();
            Rotation2d targetRotation = Rotation2d.fromDegrees(degrees);

            // The error is the difference in rotation, handled by Rotation2d to be (-pi, pi)
            double errorRad = targetRotation.minus(currentRotation).getRadians();

            // Calculate the rotational rate using a P-controller
            double rotationalRateRadPerSec = errorRad * kAngleP;

            // Cap the rotational rate
            rotationalRateRadPerSec = Math.max(-maxAngularRate, Math.min(maxAngularRate, rotationalRateRadPerSec));

            System.out.println(String.format("Current: %.2f deg, Target: %.2f deg, Error: %.2f rad, Rate: %.2f rad/s",
                currentRotation.getDegrees(),
                targetRotation.getDegrees(),
                errorRad,
                rotationalRateRadPerSec));

            // Use a FieldCentric request to apply only rotation
            return new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotationalRateRadPerSec); // Rotational rate in rad/s
        });
    }

    

}