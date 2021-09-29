package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import java.util.Locale;
import java.util.Set;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.NaiveAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import java.util.TimerTask;
import java.util.Timer;

public class GyroCalculations extends TimerTask {
    
    public void run()
    {
        AngleCalculation();
    }
    
    private BNO055IMU imu;
    public volatile boolean dataReady;
    public volatile double targetAngle, currentAngle;
    private double targetDistance, currentDistance;
    private double currentX, currentY;
    public volatile double rightSpeed, leftSpeed;
    public volatile double turnError;
    public volatile Position pos;
    public volatile Velocity vel;
    
    private int dir = 0;
    private int bias = 1;
    private BNO055IMU.Parameters parameters;
    public GyroCalculations(HardwareMap hardwareMap) throws InterruptedException
    {
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        // parameters.accelerationIntegrationAlgorithm = new NaiveAccelerationIntegrator();
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        Thread.sleep(2000);
        // Start the logging of measured acceleration
        pos = new Position();
        vel = new Velocity();
        // imu.startAccelerationIntegration(new Position(), new Velocity(), 10);
    
        targetAngle = 0; currentAngle = 0;
        targetDistance = 0; currentDistance = 0;
        rightSpeed = 0; leftSpeed = 0;
        turnError = 0;
        
        Orientation currOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(currOrientation.angleUnit, currOrientation.firstAngle)) + 180;
    }
    
    private double[] samples = new double[3];
    private int sample_count = 0;
    
    
    private void DistanceUpdate()
    {
        for (int sample_count = 0; sample_count < 5; sample_count++)
        {
            Acceleration acc = imu.getAcceleration();
            
            if (Math.abs(acc.xAccel) < 0.2) acc.xAccel = 0;
            if (Math.abs(acc.yAccel) < 0.2) acc.yAccel = 0;
            if (Math.abs(acc.zAccel) < 0.2) acc.zAccel = 0;
            samples[0] += acc.xAccel;
            samples[1] += acc.yAccel;
            samples[2] += acc.zAccel;
        }
        
        vel.xVeloc = samples[0]/5;
        vel.yVeloc = samples[1]/5;
        vel.zVeloc = samples[2]/5;
        
        pos.x += vel.xVeloc;
        pos.y += vel.yVeloc;
        pos.z += vel.zVeloc;
        
    }
    
    public void SetObjectives(double angle, double distance)
    {
        targetAngle = (currentAngle + angle) % 360;
        dir = (targetAngle == (currentAngle - angle)) ? -1 : 1;
        
        targetDistance = distance;
        
        dataReady = false;
    }
    
    public void AngleCalculation()
    {
        double P = 0.005;
        double MaxSpeed = 0.2, MinSpeed = 0.1;
        Orientation currOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(currOrientation.angleUnit, currOrientation.firstAngle)) + 180;
        double angularSpeed = 0;
        turnError = (targetAngle > currentAngle) ? ((360 - targetAngle) + currentAngle) : ((360 - currentAngle) + targetAngle);
        
        if (Math.abs(turnError) < 1) 
        {
            angularSpeed = 0;
            dataReady = true;
        }
        else
        {
            angularSpeed = P*turnError;
            if (Math.abs(angularSpeed) < MinSpeed)
                angularSpeed = MinSpeed * Math.signum(angularSpeed);
            else if(Math.abs(angularSpeed) > MaxSpeed)
                angularSpeed = MaxSpeed * Math.signum(angularSpeed);
        }
        
        leftSpeed = -angularSpeed * dir;
        rightSpeed = angularSpeed * dir;
    }
}