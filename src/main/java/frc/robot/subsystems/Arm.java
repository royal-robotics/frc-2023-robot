package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

public class Arm extends SubsystemBase {
    private DoubleSolenoid m_gripSolenoid; //left
    private DoubleSolenoid m_angleSolenoid; //right
    private TalonSRX m_leftMotor;
    private TalonSRX m_rightMotor;
    private double m_motorSpeed;
    private DoubleSolenoid.Value m_solenoidGrip;
    private DoubleSolenoid.Value m_solenoidAngle;
    private PIDController m_pid;
    private Encoder m_encoder; 
    private double m_pidValue;
    private GenericEntry speedOverride;
    private GenericEntry speedOverrideEntry;

    public Arm(){

        int limitPeak = 40; //configPeakCurrentLimit()
        int limitDuration = 2000; //configPeakCurrentDuration()
        int continuousCurrent = 30; //ContinuousCurrentLimit()
        double pidKp = 50;
        
        m_gripSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 1);
        m_angleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 2);
        m_leftMotor = new TalonSRX(10);
        m_rightMotor = new TalonSRX(11);
        m_leftMotor.enableCurrentLimit(true);
        m_leftMotor.configPeakCurrentLimit(limitPeak);
        m_leftMotor.configPeakCurrentDuration(limitDuration);
        m_leftMotor.configContinuousCurrentLimit(continuousCurrent);
        m_rightMotor.configPeakCurrentLimit(limitPeak);
        m_rightMotor.configPeakCurrentDuration(limitDuration);
        m_rightMotor.configContinuousCurrentLimit(continuousCurrent);
        m_rightMotor.enableCurrentLimit(true);
        m_motorSpeed = 0;
        m_solenoidGrip = DoubleSolenoid.Value.kReverse;
        m_solenoidAngle = DoubleSolenoid.Value.kReverse;
        m_pid = new PIDController(pidKp, 0, 0);
        m_encoder = new Encoder(10, 11,true);

        // Distance per pulse:
        //   1 motor rotation / 256 pulses
        //   11 output rotations / 150 motor rotations
        //   1.125*PI inches / 1 output rotation
        m_encoder.setDistancePerPulse((11.0 * 1.125 * Math.PI) / (256.0 * 150.0));
        m_encoder.reset();

        m_pidValue = 0;
        m_pid.setSetpoint(0);

        ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
        speedOverride = armTab.add("Speed Override", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
        speedOverrideEntry = armTab.add("Lower Speed", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -1.0, "max", 1.0, "block increment", 0.01))
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();
    }

    @Override
    public void periodic(){
        m_gripSolenoid.set(m_solenoidGrip);
        m_angleSolenoid.set(m_solenoidAngle);
        m_pidValue = -m_pid.calculate(m_encoder.getDistance());

        if (speedOverride.getBoolean(false)) {
            // to-do: change this to a position override instead of a speed override
            m_leftMotor.set(TalonSRXControlMode.PercentOutput, speedOverrideEntry.getDouble(0.0));
            m_rightMotor.set(TalonSRXControlMode.PercentOutput, -speedOverrideEntry.getDouble(0.0));
        } else {
            m_leftMotor.set(TalonSRXControlMode.PercentOutput, m_pidValue); // to-do: change m_motorSpeed to m_pidValue
            m_rightMotor.set(TalonSRXControlMode.PercentOutput, -m_pidValue); // to-do: change m_motorSpeed to m_pidValue
        }
    }

    public void setMotorSpeed(double speed){  //change later to calculate setpoint (because motorspeed isn't used here)
        m_motorSpeed = speed;
    }

    public void setSolenoidGrip(DoubleSolenoid.Value grip){
        m_solenoidGrip = grip;
    }

    public void setSolenoidAngle(DoubleSolenoid.Value angle){
        m_solenoidAngle = angle;  
    }

    public double getEncoder() {
        return m_encoder.getDistance(); //maxes out at -1.95398
    }

    public void zeroArmEncoder() {
        m_encoder.reset();
    }

    public double getPidValue() {
        return m_pidValue;
    }

    public double getSetpoint() {
        return m_pid.getSetpoint();
    }

    public void setSetpoint(double setpoint) {
        m_pid.setSetpoint(setpoint);
    }

    public boolean atSetpoint() {
        return Math.abs(m_pid.getSetpoint() - m_encoder.getDistance()) < 0.25;
    }
    public DoubleSolenoid.Value getSolenoidGrip(){
        return m_solenoidGrip;
    }

    public DoubleSolenoid.Value getSolenoidAngle(){
        return m_solenoidAngle;
    }
}
