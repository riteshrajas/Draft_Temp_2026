// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // Field-centric facing drive for default command
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // Store the last target direction to maintain it when stick is released
    private Rotation2d lastTargetDirection = new Rotation2d();



    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // Default command: drive translation with left stick, rotation with right stick
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                // P-controller for rotation
                double kAngleP = 5; // TUNE THIS
                double maxAngularRate = Math.PI; // TUNE THIS

                // Update target direction if right stick is moved
                double rightX = -joystick.getRightX();
                double rightY = -joystick.getRightY();
                if (Math.hypot(rightX, rightY) > 0.7) { // Deadband check - increased to prevent accidental changes
                    // Calculate joystick angle and convert to 0-360
                    double joystickAngleDegrees = Math.toDegrees(Math.atan2(rightY, rightX));
                    double normalizedAngle = (joystickAngleDegrees + 360) % 360;

                    // Determine which 10-degree zone the angle is in
                    int zoneIndex = (int) (normalizedAngle / 10);

                    // Calculate the center of that zone
                    double zoneCenterDegrees = zoneIndex * 10.0 + 5.0;

                    // Set the target direction to the center of the zone
                    lastTargetDirection = Rotation2d.fromDegrees(zoneCenterDegrees);
                }

                // Calculate error and rotational rate
                Rotation2d currentRotation = drivetrain.getState().Pose.getRotation();
                double errorRad = lastTargetDirection.minus(currentRotation).getRadians();
                double rotationalRateRadPerSec = errorRad * kAngleP;
                rotationalRateRadPerSec = Math.max(-maxAngularRate, Math.min(maxAngularRate, rotationalRateRadPerSec));

                return drive
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(rotationalRateRadPerSec);
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );




        // Press A to rotate robot to 90 degrees
        joystick.a().whileTrue(getRotationDegreesCommand(90));

        // Removed B mapping

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
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

    public Command getRotationRadiansCommand(double radians) {
        /* Create a command to rotate the robot to a specific angle in radians with debug output */
        return drivetrain.applyRequest(() -> {
            double currentAngle = drivetrain.getState().Pose.getRotation().getRadians();
            System.out.println("[DEBUG] Current robot angle (rad): " + currentAngle);
            System.out.println("[DEBUG] Requested rotation target (rad): " + radians);
            return new SwerveRequest.FieldCentricFacingAngle()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withTargetDirection(Rotation2d.fromRadians(radians));
        });
    }


}