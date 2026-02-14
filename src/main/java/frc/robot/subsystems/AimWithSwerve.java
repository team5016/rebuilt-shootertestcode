package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.pid;
import frc.robot.LimelightHelpers;

public class AimWithSwerve extends SubsystemBase{
    
    

    public AimWithSwerve(){
        LimelightHelpers.setPipelineIndex("limelight-turret", 0);
    }
    public Command AimWithSwerve(){
        return runEnd(
            () -> {

            }, // Action while running
            () -> {

            }
        ).withName("Turn Turret Dish With PID & AprilTag");
    }
}
