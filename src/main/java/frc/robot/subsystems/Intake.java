package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

public class Intake extends SubsystemBase {
    private DoubleSolenoid m_bottomSolenoid;
    private DoubleSolenoid m_topSolenoid;
    private TalonSRX m_lowerMotor; //left
    private TalonSRX m_upperMotor; //right
    private double m_lowerMotorSpeed;
    private double m_upperMotorSpeed;
    private DoubleSolenoid.Value m_solenoidBottomValue;
    private DoubleSolenoid.Value m_solenoidTopValue;
    private GenericEntry speedOverride;
    private GenericEntry lowerSpeedOverride;
    private GenericEntry upperSpeedOverride;

    public Intake(){
        m_bottomSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 3);
        m_topSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 0);
        m_lowerMotor = new TalonSRX(9);
        m_upperMotor = new TalonSRX(8);
        m_lowerMotorSpeed = 0;
        m_upperMotorSpeed = 0;
        m_solenoidBottomValue = DoubleSolenoid.Value.kReverse;
        m_solenoidTopValue = DoubleSolenoid.Value.kReverse;
        ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
        speedOverride = intakeTab.add("Speed Override", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
        lowerSpeedOverride = intakeTab.add("Lower Speed", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -1.0, "max", 1.0, "block increment", 0.01))
            .withPosition(0, 1)
            .withSize(2, 1)
            .getEntry();
        upperSpeedOverride = intakeTab.add("Upper Speed", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -1.0, "max", 1.0, "block increment", 0.01))
            .withPosition(0, 2)
            .withSize(2, 1)
            .getEntry();
    }

    @Override
    public void periodic(){
        m_bottomSolenoid.set(m_solenoidBottomValue);
        m_topSolenoid.set(m_solenoidTopValue);
        if (speedOverride.getBoolean(false))
        {
            m_lowerMotor.set(TalonSRXControlMode.PercentOutput, -lowerSpeedOverride.getDouble(0.0)); //Negative value normally intakes. Invert to set positive to intake.
            m_upperMotor.set(TalonSRXControlMode.PercentOutput, -upperSpeedOverride.getDouble(0.0)); //Negative value normally intakes. Invert to set positive to intake.
        }
        else
        {
            m_lowerMotor.set(TalonSRXControlMode.PercentOutput, m_lowerMotorSpeed);
            m_upperMotor.set(TalonSRXControlMode.PercentOutput, m_upperMotorSpeed);
        }
    }

    public void setLowerMotorSpeed(double speed) {
        m_lowerMotorSpeed = speed;
    }

    public void setUpperMotorSpeed(double speed) {
        m_upperMotorSpeed = speed;
    }

    public void setMotorSpeed(double speed){
        setLowerMotorSpeed(speed);
        setUpperMotorSpeed(speed);
    }
    
    public void setBottomSolenoidValue(DoubleSolenoid.Value value) {
        m_solenoidBottomValue = value;
    }

    public void setTopSolenoidValue(DoubleSolenoid.Value value) {
        m_solenoidTopValue = value;
    }

    public void setSolenoidValue(DoubleSolenoid.Value value) {
        setBottomSolenoidValue(value);
        setTopSolenoidValue(value);
    }
}
