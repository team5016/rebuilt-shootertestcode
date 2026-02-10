package frc.robot;

public class pid {
    private double totalError = 0.0;
    private double lastError = 0.0;
    private double derivative;
    private double i;
    public double PID(double error, double deltaTime, double deadSpot, double kP, double kI, double kD) {
        totalError += error * deltaTime;
        derivative = ((error - lastError) / deltaTime);
        lastError = error;
        if(Math.abs(error) < deadSpot){
            return 0;
        }
        return ((kP * error) + (kI * totalError) + kD * (derivative));
    }
}
