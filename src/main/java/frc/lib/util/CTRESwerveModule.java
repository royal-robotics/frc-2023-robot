package frc.lib.util;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class CTRESwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;

    private final CTREConfigs ctreConfigs;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

    public CTRESwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        this.ctreConfigs = new CTREConfigs();

        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = desiredState.angle;

        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), SwerveConstants.angleGearRatio));
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), SwerveConstants.angleGearRatio));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), SwerveConstants.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(SwerveConstants.angleMotorInvert);
        mAngleMotor.setNeutralMode(SwerveConstants.ctreAngleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(SwerveConstants.driveMotorInvert);
        mDriveMotor.setNeutralMode(SwerveConstants.ctreDriveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio),
            getAngle()
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio),
            getAngle()
        );
    }
}