package frc.robot.subsystems;

import java.util.Collections;
import java.util.LinkedList;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj.AnalogInput;
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
    public double m_spinMultiplier;
    public AnalogInput sonicSensorRange;
    public LinkedList<Double> voltageReadings;

    public static final double distanceToVoltage = 2.911;

    public Swerve(Visions visions) {
        gyro = new Pigeon2(Constants.Drivebase.pigeonID);
        s_Visions = visions;
        sonicSensorRange = new AnalogInput(5);
        voltageReadings = new LinkedList<Double>();
        m_speedMultiplier = Constants.fastMode;
        m_spinMultiplier = Constants.slowSpin;
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
        double m_rotate = rotation * m_speedMultiplier * m_spinMultiplier;

        SwerveModuleState[] swerveModuleStates =
            Constants.Drivebase.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    m_speed.getX(), 
                                    m_speed.getY(), 
                                    m_rotate, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    m_speed.getX(), 
                                    m_speed.getY(), 
                                    m_rotate)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for(CTRESwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }  
    
    public void setStableModuleStates() {
        SwerveModuleState[] swerveModStates = {new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4)),
            new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)),
            new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4)),
            new SwerveModuleState(0, new Rotation2d(-3 * Math.PI / 4))
        };
        
        for(CTRESwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModStates[mod.moduleNumber], true);
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

    public void setGyro(double angle) {
        gyro.setYaw(angle);
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

    public double getSonicDistance(){
        double sonicDistance = this.getVoltage() * distanceToVoltage;
        return sonicDistance;
    }

    public double getVoltage(){
        //double voltage = sonicSensorRange.getAverageVoltage();
        double voltage = sonicSensorRange.getVoltage();
        if(voltageReadings.size() >= 6){
            voltageReadings.removeFirst();
        }
        voltageReadings.add(voltage);
        
        return averageVoltageReadings();
    }

    public double averageVoltageReadings(){
        double total = 0;
        for(int i = 0; i<voltageReadings.size(); i++){
            total += voltageReadings.get(i);
        }
        double average = total/voltageReadings.size();
        double furthest = 0;
        double outlier = 0;
        for(int i = 0; i<voltageReadings.size(); i++){
            if(Math.abs(voltageReadings.get(i) - average) > furthest){
                outlier = voltageReadings.get(i);
                furthest = Math.abs(voltageReadings.get(i) - average);
            }
        }
        average = ((average * voltageReadings.size()) - outlier)/(voltageReadings.size()-1);

        return average;

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