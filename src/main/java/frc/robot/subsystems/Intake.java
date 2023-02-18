package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private DoubleSolenoid m_leftSolenoid;
    private DoubleSolenoid m_rightSolenoid;
    private TalonSRX m_lowerMotor; //left
    private TalonSRX m_upperMotor; //right
    private double m_lowerMotorSpeed;
    private double m_upperMotorSpeed;
    private DoubleSolenoid.Value m_solenoidValue;


    public Intake(){
        m_leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);
        m_rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 5);
        m_lowerMotor = new TalonSRX(9);
        m_upperMotor = new TalonSRX(8);
        m_lowerMotorSpeed = 0;
        m_upperMotorSpeed = 0;
        m_solenoidValue = DoubleSolenoid.Value.kReverse;
    }

    @Override
    public void periodic(){
        m_leftSolenoid.set(m_solenoidValue);
        m_rightSolenoid.set(m_solenoidValue);
        m_lowerMotor.set(TalonSRXControlMode.PercentOutput, m_lowerMotorSpeed);
        m_upperMotor.set(TalonSRXControlMode.PercentOutput, m_upperMotorSpeed);
    }

    public void setMotorSpeed(double lSpeed, double uSpeed){
        m_lowerMotorSpeed = lSpeed;
        m_upperMotorSpeed = uSpeed;
    }

    public void setMotorSpeed(double speed){
        setMotorSpeed(speed, speed);
    }
    

    public void setSolenoidValue(DoubleSolenoid.Value value){
        m_solenoidValue = value; 
    }

}
