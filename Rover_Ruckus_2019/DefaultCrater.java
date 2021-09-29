package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

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

@Autonomous (name = "DefaultCrater")
public class DefaultCrater extends LinearOpMode {
    
    private static DcMotor leftf, rightf, leftb, rightb, slide, collect, extend;
    private double circ = Math.PI * 4;
    private Servo hook, latch, stopper, cam, light;
    private CRServo rollers;
    private DistanceSensor frontprox, frprox, mrprox, leftprox;
    VuforiaLocalizer vuforia;
    
    private BNO055IMU imu;
    public volatile double currentAngle;
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
        
        if (hardwareMap == null)
        {
            telemetry.addLine("HardwareMap is null");
            telemetry.update();
        }
        
        // Init hardware
        leftf = hardwareMap.get(DcMotor.class, "LeftF");
        rightf = hardwareMap.get(DcMotor.class, "RightF");
        leftb = hardwareMap.get(DcMotor.class, "LeftB");
        rightb = hardwareMap.get(DcMotor.class, "RightB");
        slide = hardwareMap.get(DcMotor.class, "Slide");
        collect = hardwareMap.get(DcMotor.class, "Collection");
        extend = hardwareMap.get(DcMotor.class, "Extend");
        
        hook = hardwareMap.get(Servo.class, "LatchT");
        latch = hardwareMap.get(Servo.class, "LatchB");
        stopper = hardwareMap.get(Servo.class, "Stopper");
        cam = hardwareMap.get(Servo.class, "Cam");
        // light = hardwareMap.get(Servo.class, "Light");
        
        rollers = hardwareMap.get(CRServo.class, "Rollers");
        
        frontprox = hardwareMap.get(DistanceSensor.class, "FrontProx");
        leftprox = hardwareMap.get(DistanceSensor.class, "LeftProx");
        mrprox = hardwareMap.get(DistanceSensor.class, "MRProx");
        frprox = hardwareMap.get(DistanceSensor.class, "FRProx");
        
        leftf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        leftf.setDirection(DcMotor.Direction.FORWARD);
        rightf.setDirection(DcMotor.Direction.FORWARD);
        rightb.setDirection(DcMotor.Direction.REVERSE);
        leftb.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
        
        // Set motors to zero power braking
        leftf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collect.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        initGyro();
        
        start_angle = currentAngle();
        end_angle = currentAngle();
        currentAngle = currentAngle();
        
        TensorFlow.initVuforia(hardwareMap);
        TensorFlow.initTfod(hardwareMap);
        
        stopper.setPosition(1);
        cam.setPosition(1);
        // light.setPosition(0.7595);
        
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
        if (pwrL > 0)
            leftb.setPower(pwrL + 0.4);
        else if (pwrL < 0)
            leftb.setPower(pwrL - 0.4);
        else
            leftb.setPower(pwrL);
            
        if (pwrR > 0)
            rightb.setPower(pwrR + 0.4);
        else if (pwrR < 0)
            rightb.setPower(pwrR - 0.4);
        else
            rightb.setPower(pwrR);
            
        leftf.setPower(pwrL);
        rightf.setPower(pwrR);
        
        telemetry.addData("pwrL", pwrL);
        telemetry.addData("pwrR", pwrR);
        telemetry.update();
    }
    
    public double drive_straight(double dist, double start_angle, double end_angle) {
        
        double start_enc_left = leftf.getCurrentPosition();
        double start_enc_right = rightf.getCurrentPosition();
        start_angle = currentAngle();
        
        double dir = Math.signum(dist);
        dist = Math.abs(dist);
        Forward(0.4*dir, 0.2*dir);
        double speed = 0.4;
        
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
    
    public void DriveToWall(DistanceSensor prox) throws InterruptedException {
        while (prox.getDistance(DistanceUnit.INCH) > 22.5) {
            telemetry.addData("STATUS", "GO");
            telemetry.addData("Distance", prox.getDistance(DistanceUnit.INCH));
            telemetry.update();
            Forward(-0.5, -0.5);
        }
        telemetry.addData("STATUS", "STOPPED");
        telemetry.update();
        Forward(0,0);

        end_angle = currentAngle();
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
        final double max_speed = 0.8;
        final double min_speed = 0.4;
        do {
            Forward(mod, -mod);
            start_angle = currentAngle();
            angle_diff = end_angle - start_angle;
            
            mod = 0.012 * angle_diff;
            
            if (Math.abs(mod) > max_speed) {
                mod = max_speed * Math.signum(mod);
            } else if(Math.abs(mod) < min_speed) {
                mod = min_speed * Math.signum(mod);
            }
            
            Thread.sleep(40);
            telemetry.addData("Speed", mod);
            telemetry.update();
        } while(Math.abs(angle_diff) > 5 && !isStopRequested());
        
        Forward(0,0);
    }
    
    public double currentAngle() {
        Orientation currOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        telemetry.addData("STATUS", "orientation taken");
        telemetry.update();
        
        currentAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(currOrientation.angleUnit, currOrientation.firstAngle)) + 180;
        
        telemetry.addData("STATUS", "current angle found");
        telemetry.update();
        
        return currentAngle;
    }
    
    public void move(double dist) {
        start_angle = currentAngle();
        enc_diff = drive_straight(dist, start_angle, end_angle);
        end_angle = currentAngle();
        Forward(0,0);
    }
    
    public void turn(double deg) throws InterruptedException {
        start_angle = currentAngle();
        end_angle = (start_angle + deg) % 360;
        Correct(start_angle, end_angle);
        Forward(0,0);
    }
    
    public void tfStatusUpdate() {
        if (TensorFlow.left)
            telemetry.addData("Position", "Left");
        else if (TensorFlow.right)
            telemetry.addData("Position", "Right");
        else if (TensorFlow.center)
            telemetry.addData("Position", "center");
        else {
            telemetry.addData("Position", "not found");
            // light.setPosition(0.6645);
        }
        
        telemetry.update();
    }

    private void descend() throws InterruptedException {
        
        telemetry.addData("begin descent", "");
        telemetry.update();
        
        // Descend from landing pad
        slide.setPower(0.5);
        Thread.sleep(200);
        latch.setPosition(0);
        Thread.sleep(400);
        slide.setPower(0);
        Thread.sleep(200);
        
        telemetry.addData("huieh", "fhaeuitg");
        telemetry.update();
        
        // Unhook
        hook.setPosition(0);

        telemetry.addData("krytal hsiah", "srhdjtdt");
        telemetry.update();
        
        Thread.sleep(1000);
        
        // Down
        slide.setPower(0.3);
        Thread.sleep(1500);
        slide.setPower(0);
        
        telemetry.addData("ihsuigrr", "srr");
        telemetry.update();
        
        Thread.sleep(500);
        
        // Just get it out of the way
        hook.setPosition(1);
        
    }
    
    public void detect() throws InterruptedException {
        // CameraDevice.getInstance().setFlashTorchMode(true);

        double pos = 0;
        int cnt = 0;
        
        do {
            pos = (cam.getPosition() >= 0.75 && cam.getPosition() < 0.8) ? 0.8 : 0.75;
            cam.setPosition(pos);
            TensorFlow.detect();
            cnt++;
        } while (TensorFlow.getSize() != 2 && cnt < 4 && !isStopRequested());
        
        if (TensorFlow.tfod != null) {
            TensorFlow.tfod.shutdown();
        }
        
        telemetry.addData("# Detected", TensorFlow.getSize());
        telemetry.addData("Gold Position", TensorFlow.getPos("Gold"));
        telemetry.addData("Silver 1 Position", TensorFlow.getPos("Silver1"));
        telemetry.addData("Silver 2 Position", TensorFlow.getPos("Silver2"));
        telemetry.update();
        
        // CameraDevice.getInstance().setFlashTorchMode(false);
    }
    
    public void lowerCollect(boolean down) throws InterruptedException {
        if (down) {
            collect.setPower(0.8);
            Thread.sleep(500);
            collect.setPower(0);
            extend.setPower(1);
            Thread.sleep(450);
            extend.setPower(0);
            Thread.sleep(1000);
        } else {
            extend.setPower(-1);
            Thread.sleep(450);
            extend.setPower(0);
            collect.setPower(-0.8);
            Thread.sleep(600);
            collect.setPower(0);
        }
    }

    public void grabCube(String pos) throws InterruptedException {
        
        // light.setPosition(0.6845);
        
        lowerCollect(true);

        end_angle = currentAngle();
        
        if (pos.equals("left")) {

            // Forward just a little bit to get away from the lander
            move(0.3);
            turn(25);
            Thread.sleep(50);
            move(2.7);
            
            rollers.setPower(-1);
            Thread.sleep(1200);
            rollers.setPower(0);
            
            lowerCollect(false);
            
            Thread.sleep(50);
            move(3);
            Thread.sleep(50);

        } else if (pos.equals("right")) {

            // Forward just a little bit to get away from the lander
            move(0.2);
            turn(-20);
            Thread.sleep(50);
            move(2.7);
            
            rollers.setPower(-1);
            Thread.sleep(1200);
            rollers.setPower(0);
            
            lowerCollect(false);
            
            Thread.sleep(50);
            move(3);
            Thread.sleep(50);

        } else if (pos.equals("center")) {
            
            move(3);
            
            rollers.setPower(-1);
            Thread.sleep(1200);
            rollers.setPower(0);
            
            lowerCollect(false);
            
            Thread.sleep(50);
            move(3);
            Thread.sleep(50);
        }
        lowerCollect(true);
    }

/*
--------------------------------------------------------------------------------------------------------------------------
                                MOVEMENT CODE (AUTONPATH)
--------------------------------------------------------------------------------------------------------------------------
*/
    
    private void AutonPath() throws InterruptedException {
        
        // This is our primary program. It includes everything.

        descend();

        // backward
        move(-0.2);
        
        detect();
        
        tfStatusUpdate();
        
        // TensorFlow.center = false;
        // TensorFlow.left = false;
        // TensorFlow.right = true;

        if (TensorFlow.left) {

            grabCube("left");

        } else if (TensorFlow.right) {

            grabCube("right");
            
        } else if (TensorFlow.center) {

            grabCube("center");
            
        } else {
            
            // lowerCollect(false);

            // Forward
            move(1.5);
            
            Thread.sleep(100);
            
            turn(-120);
            
            Thread.sleep(100);
        
            DriveToWall(frontprox);
        
            turn(72);
        
            move(-6);
            
            lowerCollect(true);
        }
        
        telemetry.addData(">", "end");
        telemetry.update();
    }
}
