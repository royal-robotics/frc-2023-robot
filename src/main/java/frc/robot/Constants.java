package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.11;
    public static final double stickRotationDeadband = 0.25;

    public static final double fastMode = 1.0;
    public static final double slowMode = 0.3;

    public static final double slowSpin = 0.5;
    public static final double fastSpin = 1.0;

    public static final double cubeIntakeSpeed = -1.0;
    public static final double coneIntakeSpeed = -1.0;

    // Arm Constants
    public static final double maxArmDistance = 1.86;
    public static final double minArmDistance = 0.01;

    public static final double armTopSetpoint = maxArmDistance;
    public static final double armMiddleConeSetpoint = 1.25;
    public static final double armMiddleCubeSetpoint = 1.15;
    public static final double armBottomSetpoint = minArmDistance;

    public static final double armPidKp = 35;

    public static final double encoderAndSetPointLimit = 0.5;  //used in MoveArm.java -- limits both encoder and setpoint -- limit for if it is safe to open grip while moving arm

    public static final DoubleSolenoid.Value gripOpen =  DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value gripClose = DoubleSolenoid.Value.kReverse;
    public static final DoubleSolenoid.Value armUp = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value armDown = DoubleSolenoid.Value.kReverse;
    public static final DoubleSolenoid.Value intakeExtend = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value intakeRetract = DoubleSolenoid.Value.kReverse;

    // Scoring Alignment Constants
    public static final double yLeftCone = -0.61;
    public static final double xLeftCone = 0.845;
    public static final double yRightCone = 0.47;
    public static final double xRightCone = 0.83;

    public static final double scoreAlignDrivePidKp = 2.0;
    public static final double scoreAlignAnglePidKp = 4.0;

    // Human-Player Station Alignment Constants
    public static final double hpVisionZSetpoint = 0.70;
    public static final double hpSonicZSetpoint = 0.65;


    public static final class Container {
        public static final int translationAxis = 1;
        public static final int strafeAxis = 0;
        public static final int rotationAxis = 4; 
        public static final int intakeTranslationAxis = 1; //up and down on left joystick
        public static final int armTranslationAxis = 5; //up and down on right joystick
    }

    public static final class Drivebase {
        public static final int pigeonID = 0;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
        public static final double rotationValMultiplier = 0.03;
        public static final double rotationValCap = 0.6;
        /* Drivetrain Constants */
        public static final double trackWidth = 0.495;
        public static final double wheelBase = 0.495;

        public static final double frontLeftAngle = -31.92901611328125;
        public static final double frontRightAngle = 389.959716796875;
        public static final double backLeftAngle = 122.12677001953126;
        public static final double backRightAngle = 124.03564453125;

        public static final double chargeStationWheelSpeed = 0.3;

        /* PID Controller Gains */

        /* Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final SwerveModuleConstants Mod0 = //TODO: This must be tuned to specific robot
            new SwerveModuleConstants(
                2, /* Front Left Drive Motor ID */
                0, /* Front Left Angle Motor ID */
                3, /* Front Left Encoder ID */
                Rotation2d.fromDegrees(frontLeftAngle)); /* Front Left Angle Offset */

        /* Front Right Module - Module 1 */
        public static final SwerveModuleConstants Mod1 = //TODO: This must be tuned to specific robot
            new SwerveModuleConstants(
                1, /* Front Right Drive Motor ID */
                7, /* Front Right Angle Motor ID */
                2, /* Front Right Encoder ID */
                Rotation2d.fromDegrees(frontRightAngle)); /* Front Right Angle Offset */

        /* Back Left Module - Module 2 */
        public static final SwerveModuleConstants Mod2 = //TODO: This must be tuned to specific robot
            new SwerveModuleConstants(
                3, /* Back Left Drive Motor ID */
                4, /* Back Left Angle Motor ID */
                0, /* Back Left Encoder ID */
                Rotation2d.fromDegrees(backLeftAngle)); /* Back Left Angle Offset */

        /* Back Right Module - Module 3 */
        public static final SwerveModuleConstants Mod3 = //TODO: This must be tuned to specific robot
            new SwerveModuleConstants(
                6, /* Back Right Drive Motor ID */
                5, /* Back Right Angle Motor ID */
                1, /* Back Right Encoder ID */
                Rotation2d.fromDegrees(backRightAngle)); /* Back Right Angle Offset */
    }

    public static final class Auto { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 12;
        public static final double kPYController = 12;
        public static final double kPThetaController = 11;
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
