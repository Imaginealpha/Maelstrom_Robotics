package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import java.util.Optional;
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

@Autonomous (name = "Red Team Depot Side")
public class Red_Team_Depot_Side extends LinearOpMode {
    
    private DcMotor leftf, leftb, rightf, rightb, left_slide, right_slide, rollers, extend;
    private Servo grab, left_clamp, right_clamp;
    private ColorSensor color1, color2;
    private DistanceSensor dist1, dist2;
    
    private double circ = Math.PI * 4;
    BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    
    private double start_angle, end_angle, enc_diff;

    @Override
    public void runOpMode() throws InterruptedException {
        
        Init(hardwareMap);
        
        // assert imu == null;
        
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
    
    public void initGyro() throws InterruptedException {
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated()) {
            Thread.sleep(100); // Put in small delay for HW to init
        }
        
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
        
        // telemetry.addData("pwrL", pwrL);
        // telemetry.addData("pwrR", pwrR);
        // telemetry.update();
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
        
        Forward(0.8*dir, 0.8*dir);
        double speed = 0.8;
        
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
    
    public void DriveToBlock() throws InterruptedException {
        while (dist1.getDistance(DistanceUnit.INCH) > 5 && dist2.getDistance(DistanceUnit.INCH) > 5) {
            telemetry.addData("STATUS", "GO");
            telemetry.addData("Distance 1", dist1.getDistance(DistanceUnit.INCH));
            telemetry.addData("Distance 2", dist2.getDistance(DistanceUnit.INCH));
            telemetry.update();
            if (dist1.getDistance(DistanceUnit.INCH) < 15)
                Forward(0.3, 0.3);
            else if (dist1.getDistance(DistanceUnit.INCH) < 20)
                Forward(0.5, 0.5);
            else
                Forward(0.8, 0.8);
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
        Orientation currOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        // telemetry.addData("STATUS", "orientation taken");
        // telemetry.addData("Second Angle", currOrientation.secondAngle);
        // telemetry.update();
        
        double cAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(currOrientation.angleUnit, currOrientation.firstAngle)) + 180;
        
        // telemetry.addData("STATUS", "current angle found");
        // telemetry.update();
        
        return cAngle;
    }
    
    public void move(double dist) throws InterruptedException {
        start_angle = currentAngle();
        enc_diff = drive_straight(dist, start_angle, end_angle);
        end_angle = currentAngle();
        Forward(0,0);
    }
    
    public void turn(double deg) throws InterruptedException {
        Correct(currentAngle(), deg + 180);
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
    
    public void ext(double pwr, double sec) throws InterruptedException {
        extend.setPower(pwr);
        Thread.sleep((long)(sec*1000));
        extend.setPower(0);
    }
    
    public void collect() throws InterruptedException {
        rl(0.7, 0.75);
        ext(1, 0.6);
        grab.setPosition(1);
        Thread.sleep(200);
        rl(-0.3, 0.5);
        grab.setPosition(0.1);
        ext(-0.6, 0.25);
    }
    
    public void drop(double sec) throws InterruptedException {
        ext(0.6, sec);
        grab.setPosition(1);
        rl(0.6, sec);
        ext(-0.6, sec);
    }
    
    public void detect() throws InterruptedException {
        int red = 30, blue = 30, green = 30, alpha = 130, it = 0;
        while ((red < 80 && red > 20 || blue < 80 && blue > 20
                || green < 80  && green > 25 || alpha < 170 && alpha > 60) && it < 6) {
            red = 0;
            blue = 0;
            green = 0;
            alpha = 0;
            
            for (int i = 0; i < 10; i++) {
                red += color2.red();
                blue += color2.blue();
                green += color2.green();
                alpha += color2.alpha();
            }
            red /= 10;
            blue /= 10;
            green /= 10;
            alpha /= 10;
            
            telemetry.addData("Color Sensor Right Blue Value", blue);
            telemetry.addData("Color Sensor Right Red Value", red);
            telemetry.addData("Color Sensor Right Green Value", green);
            telemetry.addData("Color Sensor Right Alpha Value", alpha);
            telemetry.addData("Iteration", it);
            telemetry.update();
            sidle(1, 0.5);
            Correct(currentAngle(), 180);
            DriveToBlock();
            Correct(currentAngle(), 180);
            it++;
        }
        sidle(-0.5, 0.75);
        Correct(currentAngle(), 180);
        collect();
        move(-0.5);
            telemetry.addData("Iteration", it);
            telemetry.update();
        
        switch(it) {
            case 5:
                sidle(1, 0.4);
                break;
            
            case 4:
                sidle(1, 0.8);
                break;
            
            case 3:
                sidle(1, 1.2);
                break;
                
            case 2:
                sidle(1, 1.6);
                break;
            
            case 1:
                sidle(1, 2);
                break;
                
            default:
                break;
        }
        Correct(currentAngle(), 180);
    }
/*
--------------------------------------------------------------------------------------------------------------------------
                                MOVEMENT CODE (AUTONPATH)
--------------------------------------------------------------------------------------------------------------------------
*/
    
    private void AutonPath() throws InterruptedException {
        
        /* This is our movement path.
            Commands:
                move(double dist)
                turn(double deg)
                sidle(double pwr, double sec)
                rl(double pwr, double sec)
                ext(double pwr, double sec)
                collect()
                detect()
                drop(double sec)
                DriveToBlock(proximity sensor)
                Thread.sleep(long time (ms))
        */
        
        // move(5);                 // ONLY IF DRIVETOBLOCK DOES NOT WORK
        DriveToBlock();
        Correct(currentAngle(), 180); // 180 is start angle
        
        sidle(-1, 1.5);
        Correct(currentAngle(), 180);
        DriveToBlock();
        Correct(currentAngle(), 180);
        sidle(-0.5, 0.75);
        detect();
        
        // collect();
        // move(-1);
        turn(-90);                              // turn left 90
        
        move(12.5);
        Correct(currentAngle(), 90);
        turn(0);                                // turn left 90 (now that you're at 90, go to 0)
        move(8);
        drop(0.75);
        // rollers.setPower(1);
        // Thread.sleep(1200);
        move(-6);
        // rollers.setPower(0);
        rl(-1, 0.5);
        Correct(currentAngle(), 180);
        sidle(-1, 1.5);                   // negative left, positive right
        Correct(currentAngle(), 180);
        
        telemetry.addData(">", "end");
        telemetry.update();
    }
}