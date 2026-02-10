package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.*;

public class IndexingSubsystem extends SubsystemBase{
    private SparkMax dishMotor;
    
    public IndexingSubsystem(){
        dishMotor = new SparkMax(8, MotorType.kBrushless);
    }

    public Command TurnDish(double speed) {
        return runEnd(
            () -> dishMotor.set(speed), // Action while running
            () -> dishMotor.set(0.0)  // Action when stopped/released
        ).withName("Turn Turret Dish");
    }
}
