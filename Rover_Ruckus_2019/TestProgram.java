package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class TestProgram extends LinearOpMode {
    
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

    public void runOpMode() throws InterruptedException {
        Init();
        
        telemetry.addData("Status", "ready");
        telemetry.update();
        
        waitForStart();
        
        AutonPath();
    }
    
    public void Init() {
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
        light = hardwareMap.get(Servo.class, "Light");
        
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
    
    public double currentAngle() {
        Orientation currOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        telemetry.addData("STATUS", "orientation taken");
        telemetry.update();
        
        currentAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(currOrientation.angleUnit, currOrientation.firstAngle)) + 180;
        
        telemetry.addData("STATUS", "current angle found");
        telemetry.update();
        
        return currentAngle;
    }
    
    public void lowerCollect(boolean down) throws InterruptedException {
        if (down) {
            collect.setPower(0.8);
            Thread.sleep(350);
            collect.setPower(0);
            extend.setPower(1);
            Thread.sleep(100);
            extend.setPower(0);
        } else {
            extend.setPower(-1);
            Thread.sleep(100);
            extend.setPower(0);
            collect.setPower(-0.8);
            Thread.sleep(500);
            collect.setPower(0);
        }
    }
    
    public void move(double dist) {
        start_angle = currentAngle();
        enc_diff = drive_straight(dist, start_angle, end_angle);
        end_angle = currentAngle();
        Forward(0,0);
    }
    
    public double drive_straight(double dist, double start_angle, double end_angle) {
        
        double start_enc_left = leftf.getCurrentPosition();
        double start_enc_right = rightf.getCurrentPosition();
        start_angle = currentAngle();
        
        double dir = Math.signum(dist);
        dist = Math.abs(dist);
        Forward(0.5*dir, 0.3*dir);
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
    
    public void AutonPath() throws InterruptedException {
        lowerCollect(true);
        Thread.sleep(100);
        move(1);
    }
}