package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public Arm(){
        m_gripSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 1);
        m_angleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 2);
        m_leftMotor = new TalonSRX(10);
        m_rightMotor = new TalonSRX(11);
        m_motorSpeed = 0;
        m_solenoidGrip = DoubleSolenoid.Value.kReverse;
        m_solenoidAngle = DoubleSolenoid.Value.kReverse;
        m_pid = new PIDController(0.1, 0, 0);
        m_encoder = new Encoder(10, 11);

        // Distance per pulse:
        //   1 motor rotation / 256 pulses
        //   2 output rotations / 3 motor rotations
        //   1.275*PI inches / 1 output rotation
        m_encoder.setDistancePerPulse((2.0 * 1.275 * Math.PI) / (256.0 * 3.0));
        m_encoder.reset();

        m_pidValue = 0;
        m_pid.setSetpoint(0);
    }

    @Override
    public void periodic(){
        m_gripSolenoid.set(m_solenoidGrip);
        m_angleSolenoid.set(m_solenoidAngle);
        m_leftMotor.set(TalonSRXControlMode.PercentOutput, m_motorSpeed); // to-do: change m_motorSpeed to m_pidValue
        m_rightMotor.set(TalonSRXControlMode.PercentOutput, m_motorSpeed); // to-do: change m_motorSpeed to m_pidValue
        m_pidValue = m_pid.calculate(m_encoder.getDistance());
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
        return m_encoder.getDistance();
    }

    public double getPidValue() {
        return m_pidValue;
    }
}
