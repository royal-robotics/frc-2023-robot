package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CTRESwerveModule;
import frc.lib.util.SwerveConstants;
import frc.robot.Constants;
import frc.robot.Visions;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator swerveOdometry;
    public CTRESwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public SwerveModuleState[] m_ModuleState = new SwerveModuleState[4];
    public Visions s_Visions;
    public double m_speedMultiplier;

    public Swerve(Visions visions) {
        gyro = new Pigeon2(Constants.Drivebase.pigeonID);
        s_Visions = visions;
        m_speedMultiplier = 1.0;
        gyro.configFactoryDefault();
        zeroGyro();

        
        mSwerveMods = new CTRESwerveModule[] {
            new CTRESwerveModule(0, Constants.Drivebase.Mod0),
            new CTRESwerveModule(1, Constants.Drivebase.Mod1),
            new CTRESwerveModule(2, Constants.Drivebase.Mod2),
            new CTRESwerveModule(3, Constants.Drivebase.Mod3)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDrivePoseEstimator(Constants.Drivebase.swerveKinematics, getYaw(), getModulePositions(), new Pose2d());

        m_ModuleState[0] = mSwerveMods[0].getState();
        m_ModuleState[1] = mSwerveMods[1].getState();
        m_ModuleState[2] = mSwerveMods[2].getState();
        m_ModuleState[3] = mSwerveMods[3].getState();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        Translation2d m_speed = translation.times(m_speedMultiplier);
        SwerveModuleState[] swerveModuleStates =
            Constants.Drivebase.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    m_speed.getX(), 
                                    m_speed.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    m_speed.getX(), 
                                    m_speed.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for(CTRESwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);
        
        for(CTRESwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(CTRESwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(CTRESwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Drivebase.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute() {
        for(CTRESwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public PPSwerveControllerCommand driveToPoint() {
        return new PPSwerveControllerCommand(
            s_Visions.generatePath(),
            this::getPose,
            Constants.Drivebase.swerveKinematics,
            new PIDController(Constants.Auto.kPXController, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(Constants.Auto.kPYController, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(Constants.Auto.kPThetaController, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            this::setModuleStates,
            false,  // Don't use alliance color since that should be handled by the vision generatePath
            this
        );
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getModulePositions());  
        m_ModuleState[0] = mSwerveMods[0].getState();
        m_ModuleState[1] = mSwerveMods[1].getState();
        m_ModuleState[2] = mSwerveMods[2].getState();
        m_ModuleState[3] = mSwerveMods[3].getState();
    }
}