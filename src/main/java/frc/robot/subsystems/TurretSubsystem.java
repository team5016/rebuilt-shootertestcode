package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.subsystems.*;

// MAX RPM IS 5200 FOR MOTOR.SET(1); !!!

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.pid;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;

public class TurretSubsystem extends SubsystemBase{
    private pid pid;
    private double turretYaw;
    //private SparkMax dishMotor;
    private SparkMax shooterMotor;
    private double deltaTime;
    private double wheelRadiusMeters;
    private double turretAngleDgrees;
    private CANcoder shooterEncoder;
    private double pidvalue;
    private SparkMaxConfig shooterConfig;
    private CANcoderConfiguration config;
    private double shooterEncoderPosition; 
    private double lastVelocity;
    public TurretSubsystem(){
        wheelRadiusMeters = 0.06;
        turretAngleDgrees = 30;
        deltaTime = 0.0;
        LimelightHelpers.setPipelineIndex("limelight-turret", 0);
        pid = new pid();
        shooterMotor = new SparkMax(20, MotorType.kBrushless);
        //dishMotor = new SparkMax(8, MotorType.kBrushless);
        shooterConfig = new SparkMaxConfig();
        shooterEncoder = new CANcoder(0);
        config = new CANcoderConfiguration();
        shooterEncoderPosition = shooterEncoder.getPosition().getValueAsDouble();
        config.MagnetSensor.MagnetOffset = -shooterEncoderPosition;
        shooterEncoder.getConfigurator().apply(config);
    }
    // public Command TurnDishLeft() {
    //     return runEnd(
    //         () -> dishMotor.set(-0.1), // Action while running
    //         () -> dishMotor.set(0.0)  // Action when stopped/released
    //     ).withName("Turn Turret Dish Left");
    // }

    // public Command TurnDishRight() {
    //     return runEnd(
    //         () -> dishMotor.set(0.1), // Action while running
    //         () -> dishMotor.set(0.0)  // Action when stopped/released
    //     ).withName("Turn Turret Dish Right");
    // }

    // public Command TurnDish(double speed) {
    //     return runEnd(
    //         () -> dishMotor.set(speed), // Action while running
    //         () -> dishMotor.set(0.0)  // Action when stopped/released
    //     ).withName("Turn Turret Dish");
    // }



    // The bottom is very useful do not delete

    // public Command Xhoming(){
    //     return runEnd(
    //         () -> {
    //             deltaTime++;
    //             dishMotor.set(pid.PID(LimelightHelpers.getTX("limelight-turret"),0.0,deltaTime,0,0.006,0,0.00125));
    //         }, // Action while running
    //         () -> {
    //             dishMotor.set(0.0);
    //             deltaTime = 0;
    //         }
    //     ).withName("Turn Turret Dish With PID & AprilTag");
    // }

    public double findingRPM(SparkMax motor, double targetRPM){
        deltaTime=0.03;
        //System.out.println("Velocity: "+(motor.getEncoder().getVelocity()));     Prints Speed
        System.out.println("RPM: " + Math.abs(shooterMotor.getEncoder().getVelocity()));
        double feedback = pid.PID(Math.abs(targetRPM) - shooterMotor.getEncoder().getVelocity(),deltaTime,0,0.00025,0.0000025,0.000025);
        double feedforward = targetRPM/5676;
        motor.set(feedback + feedforward);
        return feedback;
    }

    public Command setRPMWithPID(double targetRPM){
        return runEnd(
            () -> {
                if (lastVelocity == 0){
                    lastVelocity = targetRPM;
                }
                lastVelocity = findingRPM(shooterMotor,targetRPM);
            },
            () -> {
                lastVelocity = 0;
                shooterMotor.set(0.0);
                deltaTime = 0;
            }
        ).withName("Reach The Shooting Speed With PID");
    }

    public double getDistance(){
        return 47.858586*Math.pow(LimelightHelpers.getTA(getName()),-0.495803); 
    }
}
