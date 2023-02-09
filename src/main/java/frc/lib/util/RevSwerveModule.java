package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class RevSwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;
    private CANCoder angleEncoder;

    private RelativeEncoder mAngleMotorEncoder;
    private RelativeEncoder mDriveMotorEncoder;
    private SparkMaxPIDController mAngleMotorController;
    private SparkMaxPIDController mDriveMotorController;

    private final CTREConfigs ctreConfigs;

    public RevSwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        this.ctreConfigs = new CTREConfigs();

        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        mAngleMotorEncoder = mAngleMotor.getEncoder();
        mAngleMotorController = mAngleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        mDriveMotorEncoder = mDriveMotor.getEncoder();
        mDriveMotorController = mDriveMotor.getPIDController();
        configDriveMotor();
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = SwerveModuleState.optimize(desiredState, getAngle());

        setAngle(desiredState);
        setSpeed(desiredState);
    }

    private void setSpeed(SwerveModuleState desiredState) {
        mDriveMotorController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
    }

    private void setAngle(SwerveModuleState desiredState) {
        mAngleMotorController.setReference(desiredState.angle.getDegrees(), ControlType.kPosition);
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(mAngleMotorEncoder.getPosition());
    }

    public void resetToAbsolute() {
        double absolutePosition = angleEncoder.getAbsolutePosition() - angleOffset.getDegrees();
        mAngleMotorEncoder.setPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        mAngleMotor.setIdleMode(SwerveConstants.revAngleNeutralMode);
        mAngleMotor.setInverted(SwerveConstants.angleMotorInvert);
        mAngleMotor.setSmartCurrentLimit(SwerveConstants.angleContinuousCurrentLimit);
        mAngleMotor.setSecondaryCurrentLimit(SwerveConstants.anglePeakCurrentDuration);
        // Position Conversion:
        //   Multiply by gear ratio: Converts rotations of motor to rotations of module
        //   Divide by 360: Converts module rotations to module degrees
        double positionConversion = SwerveConstants.angleGearRatio / 360.0;
        mAngleMotorEncoder.setPositionConversionFactor(positionConversion);
        mAngleMotorController.setPositionPIDWrappingEnabled(true);
        /*
        PID values should be set in the REV Hardware Client and burned to flash.
        Copy the values here so we don't forget them!
        mAngleMotorController.setP(0.0);
        mAngleMotorController.setI(0.0);
        mAngleMotorController.setD(0.0);
        mAngleMotorController.setFF(0.0);
        */
    }

    private void configDriveMotor() {
        mDriveMotor.setIdleMode(SwerveConstants.revDriveNeutralMode);
        mDriveMotor.setInverted(SwerveConstants.driveMotorInvert);
        mDriveMotor.setSmartCurrentLimit(SwerveConstants.driveContinuousCurrentLimit);
        mDriveMotor.setSecondaryCurrentLimit(SwerveConstants.drivePeakCurrentDuration);
        // Position Conversion:
        //   Multiply by gear ratio: Converts rotations of motor to rotations of wheel
        //   Divide by wheel circumference: Converts rotations of wheel to distance traveled
        double positionConversion = SwerveConstants.driveGearRatio / SwerveConstants.wheelCircumference;
        mDriveMotorEncoder.setPositionConversionFactor(positionConversion);
        // Velocity Conversion
        //   Multiply by Position Conversion: Converts RPM of motor to meters per minute
        //   Divide by 60: Converts meters per minute to meters per second
        double velocityConversion = positionConversion / 60.0;
        mDriveMotorEncoder.setVelocityConversionFactor(velocityConversion);
        /*
        PID values should be set in the REV Hardware Client and burned to flash.
        Copy the values here so we don't forget them!
        mDriveMotorController.setP(0.0);
        mDriveMotorController.setI(0.0);
        mDriveMotorController.setD(0.0);
        mDriveMotorController.setFF(0.0);
        */
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            mDriveMotorEncoder.getVelocity(),
            getAngle()
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            mDriveMotorEncoder.getPosition(),
            getAngle()
        );
    }
}
