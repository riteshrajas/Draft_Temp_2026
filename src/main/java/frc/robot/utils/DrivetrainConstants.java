package frc.robot.utils;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.utils.preferences.RobotPreferences;

public class DrivetrainConstants {
    // public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double maxSpeed = 3;
    public static final double maxAngularSpeed = Math.PI;
    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public static final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public static final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public static final SwerveDriveBrake brake = new SwerveDriveBrake()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public static final SlewRateLimiter xLimiter = new SlewRateLimiter(100);
    public static final SlewRateLimiter yLimiter = new SlewRateLimiter(100);
    public static Rotation2d lastTargetDirection = new Rotation2d();
    public static final RobotPreferences prefs = RobotPreferences.getInstance();

    public static double getMaxAngularSpeed() {
        return maxAngularSpeed;
    }

    public static double getMaxSpeed() {
        return maxSpeed;
    }
}

