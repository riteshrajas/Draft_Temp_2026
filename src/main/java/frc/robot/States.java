package frc.robot;

import frc.robot.utils.motion.PIDControllerWrapper;

public class States {
    public enum ElevatorState {
        // Coral Placements
        L1(4.472),
        L2(9.3),
        L3(15.7),
        L4(28.48),
        Barge(31),
        // Algae Removal
        LowAlgae(8),
        HighAlgae(15.5),
        // General
        Zero(0.4);

        private final double height;

        ElevatorState(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }
    }
    public enum ElevatorPIDState {
        UP(0.53, 0, 0, 0.6, 0.245),
        DOWN(0.4, 0, 0, 0.6, 0.245);

        private final double kP;
        private final double kI;
        private final double kD;
        private final double kG;
        private final double kS;

        ElevatorPIDState(double kP, double kI, double kD, double kG, double kS) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kG = kG;
            this.kS = kS;
        }

        public double getKP() {
            return kP;
        }

        public double getKI() {
            return kI;
        }

        public double getKD() {
            return kD;
        }

        public double getKG() {
            return kG;
        }

        public double getKS() {
            return kS;
        }
    }

    public enum WheelsState {
        // Intake speeds
        INTAKE_SLOW(-0.1),
        INTAKE_NORMAL(-0.2),
        INTAKE_FAST(-0.5),
        // Outtake speeds
        OUTTAKE_SLOW(0.05),
        OUTTAKE_NORMAL(0.1),
        OUTTAKE_FAST(0.3),
        // Special modes
        STOP(0.0),
        HOLD(0.02); // Small holding force

        private final double speed;

        WheelsState(double speed) {
            this.speed = speed;
        }

        public double getSpeed() {
            return speed;
        }

        public boolean isIntake() {
            return speed < 0;
        }

        public boolean isOuttake() {
            return speed > 0;
        }

        public boolean isStopped() {
            return Math.abs(speed) < 0.01;
        }
    }

    public enum ArmState {
        // Coral Placement
        L1(0.01),
        L2(0.0678),
        L3(0.0678),
        Barge(0.022),

        // Algea Retrieval
        Algea(0.016),

        // General
        Start(0.00),
        Safe(0.04),
        AgainstReef(0.0234);

        private final double position;

        ArmState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }

        public ArmPIDState getPIDState() {
            return switch (this) {
                case Algea -> ArmPIDState.AlgeaRetrieval;
                default -> ArmPIDState.CoralPlacement;
            };
        }
    }
    public enum ArmPIDState {
        CoralPlacement(5,0,0,0,0),
        AlgeaRetrieval(6.5,0,0,0,0);

        private final double kP;
        private final double kI;
        private final double kD;
        private final double kF;
        private final double kG;

        ArmPIDState(double kP, double kI, double kD, double kF, double kG) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            this.kG = kG;
        }

        public PIDControllerWrapper getPIDController() {
            return new PIDControllerWrapper(
                "Arm_" + this.name(),
                kP,
                kI,
                kD,
                kF,
                kG
            );
        }
    }
    

    public enum Status {
        IDLE,
        MOVING,
        AT_TARGET,
        ERROR,
        EMERGENCY_STOP,
        MAINTENANCE,
        CALIBRATING,
        MOVING_UP,
        MOVING_DOWN,
        HOLDING,
        MOVING_TO_POSITION,
        HOLDING_POSITION,
        RESETTING,
        LIMIT
    }

}
