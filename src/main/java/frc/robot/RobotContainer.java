// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.framework.RobotFramework;
import frc.robot.utils.DrivetrainConstants;
import frc.robot.utils.telemetry.TelemetryManager;


public class RobotContainer extends RobotFramework{

    private final TelemetryManager logger = new TelemetryManager(DrivetrainConstants.maxSpeed);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Command> teleOpChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        teleOpChooser = TeleOpBuilder.buildTeleOpChooser("Holonomic Drive");
        SmartDashboard.putData("Teleop Mode", teleOpChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        
        drivetrain.setDefaultCommand(
            TeleOpBuilder.buildFalconDrive(DrivetrainConstants.getMaxSpeed(), DrivetrainConstants.getMaxAngularSpeed())
        );

       
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );




        // Press A to rotate robot to 90 degrees
        joystick.a().whileTrue(getRotationDegreesCommand(90));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

  



}