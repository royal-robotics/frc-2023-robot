package src.test.java;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.LinkedList;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Visions.Align;
import frc.robot.subsystems.Swerve;
import frc.robot.Visions;

class VoltageTest {
    // Visions vision;
    // Swerve swerve;

    @BeforeEach // this method will run before each test
    void setup() {
        // vision = new Visions();
        // swerve = new Swerve(vision);
    }

    @Test
    void testAverage(){
        LinkedList<Double> stuff = new LinkedList<>();
        stuff.add(1.4);
        stuff.add(0.9834);
        stuff.add(1.231);
        stuff.add(0.834234);
        stuff.add(1.11111);
        stuff.add(0.4);
        assertEquals(1.1119488,averageVoltageReadings(stuff));
        System.out.println(averageVoltageReadings(stuff));
    }

    @Test
    void testList(){
        LinkedList<Double> stuff = new LinkedList<>();
        stuff.add(1.4);
        stuff.add(0.9834);
        stuff.add(1.231);
        stuff.add(0.89);
        stuff.add(1.356);
        stuff.add(1.0);
        getVoltage(stuff, 12.3);
        System.out.println(stuff.toString());
        System.out.println(averageVoltageReadings(stuff));
    }

    public double averageVoltageReadings(LinkedList<Double> voltageReadings){
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
    public double getVoltage(LinkedList<Double> voltageReadings, double voltage){
        if(voltageReadings.size() >= 6){
            voltageReadings.removeFirst();
        }
        voltageReadings.add(voltage);
        
        return averageVoltageReadings(voltageReadings);
    }
}