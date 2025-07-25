package frc.robot.utils.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;

public class DeadbandController extends CommandXboxController {
    private double leftStickDeadband = 0.1;
    private double rightStickDeadband = 0.1;
    private double triggerDeadband = 0.05;

    public DeadbandController(int port) {
        super(port);
    }

    public DeadbandController withLeftStickDeadband(double deadband) {
        this.leftStickDeadband = deadband;
        return this;
    }

    public DeadbandController withRightStickDeadband(double deadband) {
        this.rightStickDeadband = deadband;
        return this;
    }

    public DeadbandController withTriggerDeadband(double deadband) {
        this.triggerDeadband = deadband;
        return this;
    }

    @Override
    public double getLeftX() {
        return MathUtil.applyDeadband(super.getLeftX(), leftStickDeadband);
    }
    @Override
    public double getLeftY() {
        return MathUtil.applyDeadband(super.getLeftY(), leftStickDeadband);
    }

    @Override
    public double getRightX() {
        return MathUtil.applyDeadband(super.getRightX(), rightStickDeadband);
    }
@Override
    public double getRightY() {
        return MathUtil.applyDeadband(super.getRightY(), rightStickDeadband);
    }
@Override
    public double getLeftTriggerAxis() {
        return MathUtil.applyDeadband(super.getLeftTriggerAxis(), triggerDeadband);
    }
@Override
    public double getRightTriggerAxis() {
        return MathUtil.applyDeadband(super.getRightTriggerAxis(), triggerDeadband);
    }
}
