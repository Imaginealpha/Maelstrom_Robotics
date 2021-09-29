package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import java.util.Set;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.vuforia.CameraDevice;

import java.util.List;
import java.util.Timer;
import java.util.Objects;

@Autonomous (name = "Blue Team Foundation Side")
public class Blue_Team_Foundation_Side extends LinearOpMode {
    
    private DcMotor leftf, leftb, rightf, rightb, left_slide, right_slide, rollers, extend;
    private Servo grab, left_clamp, right_clamp;
    private ColorSensor color1, color2;
    private DistanceSensor dist1, dist2;
    
    private double circ = Math.PI * 4;
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    
    private double start_angle, end_angle, enc_diff;

    @Override
    public void runOpMode() throws InterruptedException {
        
        Init(hardwareMap);
        
        assert imu == null;
        
        waitForStart();
        
        AutonPath();
    }
    
/*
--------------------------------------------------------------------------------------------------------------------------
                                METHODS
--------------------------------------------------------------------------------------------------------------------------
*/

    public void Init(HardwareMap hardwareMap) throws InterruptedException {
        
        // Init hardware
        leftf = hardwareMap.get(DcMotor.class, "Front Left");
        rightf = hardwareMap.get(DcMotor.class, "Front Right");
        leftb = hardwareMap.get(DcMotor.class, "Back Left");
        rightb = hardwareMap.get(DcMotor.class, "Back Right");
        left_slide = hardwareMap.get(DcMotor.class, "Left Slide");
        right_slide = hardwareMap.get(DcMotor.class, "Right Slide");
        rollers = hardwareMap.get(DcMotor.class, "Rollers");
        extend = hardwareMap.get(DcMotor.class, "Extend");
        grab = hardwareMap.get(Servo.class, "Grab");
        left_clamp = hardwareMap.get(Servo.class, "Left Clamp");
        right_clamp = hardwareMap.get(Servo.class, "Right Clamp");
        color1 = hardwareMap.get(ColorSensor.class, "Color 1");
        color2 = hardwareMap.get(ColorSensor.class, "Color 2");
        dist1 = hardwareMap.get(DistanceSensor.class, "Distance 1");
        dist2 = hardwareMap.get(DistanceSensor.class, "Distance 2");
        
        leftf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        leftf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // WHEEL DIRECTIONS OPPOSITE FROM TELEOP
        rightf.setDirection(DcMotor.Direction.REVERSE);
        rightb.setDirection(DcMotor.Direction.REVERSE);
        leftf.setDirection(DcMotor.Direction.REVERSE);
        // leftb.setDirection(DcMotor.Direction.REVERSE);
        left_slide.setDirection(DcMotor.Direction.REVERSE);
        // right_slide.setDirection(DcMotor.Direction.REVERSE);
        extend.setDirection(DcMotor.Direction.REVERSE);
        rollers.setDirection(DcMotor.Direction.REVERSE);
        
        // Set motors to zero power braking
        leftf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        initGyro();
        
        start_angle = currentAngle();
        end_angle = currentAngle();
        
        left_clamp.setPosition(0);
        right_clamp.setPosition(1);
        grab.setPosition(0.4);
        
        telemetry.addData(">", "Ready to start");
        telemetry.update();
    }
    
    public void initGyro() {
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated());
    }
    
    public void Forward(double pwrL, double pwrR) {
        
        if (pwrL > 0) {
            leftf.setPower(pwrL + 0.4);
            leftb.setPower(pwrL + 0.4);
        } else if (pwrL < 0) {
            leftf.setPower(pwrL - 0.4);
            leftb.setPower(pwrL - 0.4);
        } else
            leftf.setPower(pwrL);
            leftb.setPower(pwrL);
            
        if (pwrR > 0) {
            rightf.setPower(pwrR + 0.4);
            rightb.setPower(pwrR + 0.4);
        } else if (pwrR < 0) {
            rightf.setPower(pwrR - 0.4);
            rightb.setPower(pwrR - 0.4);
        } else
            rightf.setPower(pwrR);
            rightb.setPower(pwrR);
            
        leftf.setPower(pwrL);
        leftb.setPower(pwrL);
        rightf.setPower(pwrR);
        rightb.setPower(pwrR);
        
        telemetry.addData("pwrL", pwrL);
        telemetry.addData("pwrR", pwrR);
        telemetry.update();
    }
    
    public double drive_straight(double dist, double start_angle, double end_angle) throws InterruptedException {
        
        double start_enc_left = leftf.getCurrentPosition();
        double start_enc_right = rightf.getCurrentPosition();
        start_angle = currentAngle();
        
        double dir = Math.signum(dist);
        dist = Math.abs(dist);
        
        leftf.setPower(0.1);
        leftb.setPower(0.1);
        rightf.setPower(0.1);
        rightb.setPower(0.1);
        
        Thread.sleep(250);
        
        Forward(0.4*dir, 0.4*dir);
        double speed = 0.6;
        
        // telemetry.addData("PwrL", 0.5*dir);
        // telemetry.addData("PwrR", 0.4*dir);
        // telemetry.update();
        
        double end_enc_left = leftf.getCurrentPosition();
        double end_enc_right = rightf.getCurrentPosition();
        double enc_diff = (Math.abs(end_enc_left - start_enc_left) + Math.abs(end_enc_right - start_enc_right))/2;
    
        double mod = 0;
        
        //1000 ticks per rev, left encoder counts are doubled
        while (enc_diff < dist/circ*1000 && !isStopRequested()) {
            
            end_enc_left = leftf.getCurrentPosition();
            end_enc_right = rightf.getCurrentPosition();
            enc_diff = (Math.abs(end_enc_left - start_enc_left) + Math.abs(end_enc_right - start_enc_right))/2;
            end_angle = currentAngle();
            mod = (end_angle - start_angle) * 0.05;
            
            if (Math.abs(mod) > .25) {
                mod = (end_angle > start_angle) ? ((360 - end_angle) + start_angle) : ((360 - start_angle) + end_angle);
                mod *= 0.05;
            }
            
            mod = (mod > 0.1) ? 0 : mod;
            Forward((speed + mod)*dir, 0.9*(speed - mod)*dir);
            
            // telemetry.addData("Target:", dist/circ*1000);
            // telemetry.addData("Current:", enc_diff);
            // telemetry.addData("LeftEnc", front_left.getCurrentPosition());
            // telemetry.addData("RightEnc", front_right.getCurrentPosition());
            // telemetry.addData("Mod", mod);
            // telemetry.addData("Angle", end_angle);
            // telemetry.update();
        }
        return enc_diff;
    }
    
    public void DriveToFoundation() throws InterruptedException {
        double distance1 = dist1.getDistance(DistanceUnit.INCH), distance2 = dist2.getDistance(DistanceUnit.INCH);
        while (distance1 > 2 && distance2 > 2) {
            distance1 = dist1.getDistance(DistanceUnit.INCH);
            distance2 = dist2.getDistance(DistanceUnit.INCH);
            telemetry.addData("STATUS", "GO");
            telemetry.addData("Distance 1", distance1);
            telemetry.addData("Distance 2", distance2);
            telemetry.update();
            if (distance1 > 5 || distance2 > 5)
                Forward(0.9, 0.9);
            if (distance1 < 5 && distance1 > 3 || distance2 < 5 && distance2 > 3)
                Forward(0.5, 0.5);
            if (distance1 < 3 || distance2 < 3)
                Forward(0.2, 0.2);
        }
        telemetry.addData("STATUS", "STOPPED");
        telemetry.update();
        Forward(0,0);
    }
    
    String formatFloat(float rate) {
        return String.format("%.3f", rate);
    }
    
    public double ToRad(double deg) {
        return (deg/180)*Math.PI;
    }
    
    void Correct(double start_angle, double end_angle) throws InterruptedException {
        
        double mod = 0;
        double angle_diff = (end_angle - start_angle);
        final double max_speed = 0.5;
        final double min_speed = 0.2;
        do {
            Forward(-mod, mod);
            start_angle = currentAngle();
            angle_diff = end_angle - start_angle;
            
            telemetry.addData("Current Angle", start_angle);
            telemetry.addData("Angle Difference", angle_diff);
            
            mod = 0.012 * angle_diff;
            
            if (Math.abs(mod) > max_speed) {
                mod = max_speed * Math.signum(mod);
            } else if(Math.abs(mod) < min_speed) {
                mod = min_speed * Math.signum(mod);
            }
            
            // Thread.sleep(10);
            // telemetry.addData("Speed", mod);
            telemetry.update();
        } while(Math.abs(angle_diff) > 5 && !isStopRequested());
        
        Forward(0,0);
    }
    
    public double currentAngle() {
        Orientation currOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        
        telemetry.addData("STATUS", "orientation taken");
        telemetry.update();
        
        double cAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(currOrientation.angleUnit, currOrientation.secondAngle)) + 180;
        
        telemetry.addData("STATUS", "current angle found");
        telemetry.update();
        
        return cAngle;
    }
    
    public void move(double dist) throws InterruptedException {
        start_angle = currentAngle();
        enc_diff = drive_straight(dist, start_angle, end_angle);
        end_angle = currentAngle();
        Forward(0,0);
    }
    
    public void turn(double deg) throws InterruptedException {
        start_angle = currentAngle();
        end_angle = (start_angle + deg) % 360;
        
        telemetry.addData("Start Angle", start_angle);
        telemetry.addData("End Angle", end_angle);
        telemetry.update();
        
        Correct(start_angle, end_angle);
        Forward(0,0);
    }
    
    public void sidle(double pwr, double sec) throws InterruptedException {
        double mod = 0.018 * Math.signum(pwr);
        leftf.setPower(0.75*pwr + mod);
        rightf.setPower(-0.75*pwr - mod);
        leftb.setPower(-pwr + mod);
        rightb.setPower(pwr - mod);
        Thread.sleep((long)(sec*1000));
        leftf.setPower(0);
        rightf.setPower(0);
        leftb.setPower(0);
        rightb.setPower(0);
    }
    
    public void rl(double pwr, double sec) throws InterruptedException {
        right_slide.setPower(pwr);
        left_slide.setPower(pwr);
        Thread.sleep((long)(sec*1000));
        right_slide.setPower(0);
        left_slide.setPower(0);
    }
/*
--------------------------------------------------------------------------------------------------------------------------
                                MOVEMENT CODE (AUTONPATH)
--------------------------------------------------------------------------------------------------------------------------
*/
    
    private void AutonPath() throws InterruptedException {
        
        /* This is our primary program. It includes everything.
            Commands:
                move(double dist)
                turn(double angle)
                sidle(double dist)
                collect()
                drop(double sec)
                movetoline()
                clamp()
                DriveToWall(proximity sensor)
                Thread.sleep(double time (ms))
        */
        double sidle_angle = currentAngle();
        move(8);
        // DriveToFoundation();
        Correct(currentAngle(), sidle_angle);
        sidle(-1, 0.6);
        Correct(currentAngle(), sidle_angle);
        
        Forward(0.2, 0.2);
        Thread.sleep(700);
        right_clamp.setPosition(0);
        left_clamp.setPosition(1);
        Thread.sleep(500);
        Forward(0, 0);
        Thread.sleep(300);
        move(-8.75);
        left_clamp.setPosition(0);
        right_clamp.setPosition(1);
        Correct(currentAngle(), sidle_angle);
        
        // sidle(0.5, 3*2.4); // 3 seconds is one block @ 0.5
        // Thread.sleep(2000);
        
        sidle(1, 2.5);
        Correct(currentAngle(), sidle_angle);
        
        // move(12);
        // Correct(currentAngle(), sidle_angle);
        // sidle(-1, 2);
        // Correct(currentAngle(), sidle_angle);
        // move(-5.5);
        // Correct(currentAngle(), sidle_angle);
        // sidle(1, 1.5);
        // Correct(currentAngle(), sidle_angle);
        // move(-6.5);
        // Correct(currentAngle(), sidle_angle);
        // sidle(0.75, 1.5);
        // Correct(currentAngle(), sidle_angle);
        
        telemetry.addData(">", "end");
        telemetry.update();
    }
}