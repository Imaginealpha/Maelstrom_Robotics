package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Timer;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import com.qualcomm.hardware.bosch.*;
import java.util.Timer;

@Disabled
@Autonomous (name="AutonRLPictograph")
public class AutonRLPictograph extends LinearOpMode
{
    private DcMotor front_left, front_right, back_left, back_right, middle_wheel, glyph_pulley;
    private double circ = Math.PI * 4;
    
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;
    
    private Servo jewelArm,leftArm,rightArm, jewelTurn;
    private ColorSensor colorSensor;
    
    private void Init()
    {
        front_left = hardwareMap.get(DcMotor.class, "FrontLeft");
        front_right = hardwareMap.get(DcMotor.class, "FrontRight");
        back_left = hardwareMap.get(DcMotor.class, "BackLeft");
        back_right = hardwareMap.get(DcMotor.class, "BackRight");
        middle_wheel = hardwareMap.get(DcMotor.class, "MiddleWheel");
        
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middle_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        front_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        
        glyph_pulley = hardwareMap.get(DcMotor.class, "GlyphPulley");
        
        jewelArm = hardwareMap.get(Servo.class, "JewelArm");
        colorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
        leftArm = hardwareMap.get(Servo.class, "LeftArm");
        rightArm = hardwareMap.get(Servo.class, "RightArm");
        jewelTurn = hardwareMap.get(Servo.class, "JewelTurn");
        
        
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
        hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Achjvg3/////AAAAGfCii8VvD0iZksIjgMryq22ClRyyueYv3yEYuuxJaFrBiWzG5D0HuEOWIcTTl0+9uEb2Q3XL2nE8MzCKlrdeYEcJ7T2bUTuda7pp3XZVAbQx32sSPz1aVTcwpKqfNlUGDoZUbR8/XZis+6m6LjlyVAJX0NiqM/bVyPtXVegYl2oFdkYTi4iCrhQ5VAhQCkOgf30h9omjwOH+pmyZzYlZ+CvXRswAlnOA6OJj1o6ae9x3AfgeqmmXo2PMGng5mDC/vzSkk9YNsoblfzj8ukzsgmPaBZ05ry4ybqezejRsh7PNGbOu3GmKVXvIppaHtuDONKbkw3TX62hvLlgR+0o8tZngwHmogzLykrAnRSy8ylx4";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }
    
    String formatFloat(float rate) {
        return String.format("%.3f", rate);
    }
    
    private double ToRad(double deg)
    {
        return (deg/180)*Math.PI;
    }
    
    public void Forward(double pwrL, double pwrR)
    {
        front_left.setPower(pwrL);
        back_left.setPower(pwrL);
        back_right.setPower(pwrR);
        front_right.setPower(pwrR);
    }
    
    double drive_straight(double dist, double start_angle, double end_angle, GyroCalculations gyroCalc)
    {
        double start_enc_left = front_left.getCurrentPosition();
        double start_enc_right = front_right.getCurrentPosition();
        start_angle = gyroCalc.currentAngle;
        
        double dir = Math.signum(dist);
        dist = Math.abs(dist);
        Forward(0.15*dir, 0.15*dir); 
        
        double end_enc_left = front_left.getCurrentPosition();
        double end_enc_right = front_right.getCurrentPosition();
        double enc_diff = (Math.abs(end_enc_left - start_enc_left) + Math.abs(end_enc_right - start_enc_right))/2;
    
        double mod = 0;
        
        //1000 ticks per rev, left encoder counts are doubled
        while (enc_diff < dist/circ*1000)
        {
            end_enc_left = front_left.getCurrentPosition();
            end_enc_right = front_right.getCurrentPosition();
            enc_diff = (Math.abs(end_enc_left - start_enc_left) + Math.abs(end_enc_right - start_enc_right))/2;
            end_angle = gyroCalc.currentAngle;
            mod = (end_angle - start_angle) * 0.05;
            
            if (Math.abs(mod) > .25)
            {
                
                mod = (end_angle > start_angle) ? ((360 - end_angle) + start_angle) : ((360 - start_angle) + end_angle);
                mod *= 0.05;
            }
            
            mod = (mod > 0.1) ? 0 : mod;
            Forward((0.25 + mod)*dir, (0.25 - mod)*dir);
            
            telemetry.addData("Target:", dist/circ*1000);
            telemetry.addData("Current:", enc_diff);
            telemetry.addData("LeftEnc", front_left.getCurrentPosition());
            telemetry.addData("RightEnc", front_right.getCurrentPosition());
            telemetry.addData("Mod", mod);
            telemetry.addData("Angle", end_angle);
            telemetry.update();
        }
        return enc_diff;
    }
    
    void Correct(double start_angle, double end_angle, GyroCalculations gyroCalc) throws InterruptedException
    {
        double mod = 0;
        double angle_diff = (end_angle - start_angle);
    
        do
        {   
            Forward(mod, -mod);
            end_angle = gyroCalc.currentAngle;
            
            angle_diff = end_angle - start_angle;
            
            mod = 0.0025 * angle_diff;
            telemetry.addData("SAngle", start_angle);
            telemetry.addData("EAngle", end_angle);
            telemetry.addData("Diff", angle_diff);
            telemetry.addData("Mod", mod);
            telemetry.update();
            
            if (Math.abs(mod) > 0.5)
            {
                mod = 0.5 * Math.signum(mod);
            }else if(Math.abs(mod) < 0.2)
            {
                mod = 0.2 * Math.signum(mod);
            }
            
            
            Thread.sleep(50);
        }while(Math.abs(angle_diff) > 2);
        Forward(0,0);
    }
    
    // private void RightPath (GyroCalculations gyroCalc) throws InterruptedException
    // {
    //     double start_angle = gyroCalc.currentAngle;
    //     middle_wheel.setPower(0.5); Thread.sleep(300); middle_wheel.setPower(0); Thread.sleep(1000);
    //     double end_angle = gyroCalc.currentAngle;
    //     jewelArm.setPosition(0.7);
    //     //Forward
    //     start_angle = gyroCalc.currentAngle;
    //     double enc_diff = drive_straight(20, start_angle, end_angle, gyroCalc);
    //     end_angle = gyroCalc.currentAngle;
    //     Forward(0,0);
    //     Thread.sleep(1000);
        
    //     //90 Degree turn
    //     start_angle = gyroCalc.currentAngle;
    //     end_angle = (start_angle - 78) % 360;
    //     Correct(end_angle, start_angle, gyroCalc);
    //     Forward(0,0);
    //     Thread.sleep(1000);
        
    //     //Forward
    //     start_angle = gyroCalc.currentAngle;
    //     enc_diff = drive_straight(30, start_angle, end_angle, gyroCalc); //Adjust distance
    //     end_angle = gyroCalc.currentAngle;
    //     Forward(0,0);
    //     // Correct(start_angle+5, end_angle, gyroCalc); //Adjust correction factor
    //     Thread.sleep(1000);
        
    //     //MiddleWheel sideways 1250ms
    //     middle_wheel.setPower(-0.5); Thread.sleep(1600); middle_wheel.setPower(0); Thread.sleep(1000);
        
    // }
    
    private void AutonPath (GyroCalculations gyroCalc) throws InterruptedException
    {
        // Return jewel arm to start position
        jewelTurn.setPosition(0.5);
        jewelArm.setPosition(0.7);
        Thread.sleep(500);
        
        //Move to pictograph
        double start_angle = gyroCalc.currentAngle;
        middle_wheel.setPower(-0.5); Thread.sleep(200); middle_wheel.setPower(0); Thread.sleep(1000);
        double end_angle = gyroCalc.currentAngle;
        Thread.sleep(500);
        
        // Pictograph Code
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
        } else {
            telemetry.addData("VuMark", "not visible");
        }
        telemetry.update();
        Thread.sleep(1000);

        //Forward
        start_angle = gyroCalc.currentAngle;
        double enc_diff = drive_straight(11, start_angle, end_angle, gyroCalc);
        end_angle = gyroCalc.currentAngle;
        Forward(0,0);
        // Correct(start_angle+2.5, end_angle, gyroCalc);
        Thread.sleep(500);
        
        //90 Degree turn
        start_angle = gyroCalc.currentAngle;
        end_angle = (start_angle - 78) % 360;
        Correct(end_angle, start_angle, gyroCalc);
        Forward(0,0);
        Thread.sleep(500);
        
        //Forward
        start_angle = gyroCalc.currentAngle;
        enc_diff = drive_straight(18, start_angle, end_angle, gyroCalc);
        end_angle = gyroCalc.currentAngle;
        Forward(0,0);
        //Correct(start_angle+2.5, end_angle, gyroCalc); //Maybe remove
        Thread.sleep(500);
        
        //90 Degree turn
        start_angle = gyroCalc.currentAngle;
        end_angle = (start_angle - 78) % 360;
        Correct(end_angle, start_angle, gyroCalc);
        Forward(0,0);
        Thread.sleep(500);
        
        //Forward
        start_angle = gyroCalc.currentAngle;
        enc_diff = drive_straight(20, start_angle, end_angle, gyroCalc);
        end_angle = gyroCalc.currentAngle;
        Forward(0,0);
        //Correct(start_angle+2.5, end_angle, gyroCalc); //Maybe remove
        Thread.sleep(500);
        
        //Sideways
        int side_time = 1200;
        if (vuMark == RelicRecoveryVuMark.RIGHT)
        {
            side_time = 0;
        }
        else if(vuMark == RelicRecoveryVuMark.LEFT)
        {
            side_time = 1400;
        }
        telemetry.addData("side_time", side_time);
        telemetry.update();
        middle_wheel.setPower(0.5); Thread.sleep(side_time); middle_wheel.setPower(0); Thread.sleep(500);

        //Move Forward to Place Block
        start_angle = gyroCalc.currentAngle;
        end_angle = gyroCalc.currentAngle;
        enc_diff = drive_straight(5, start_angle, end_angle, gyroCalc);
        end_angle = gyroCalc.currentAngle;
        Forward(0,0);
        
        // Open Arm Servos
        PulleyLift(-0.5, 1000);
        leftArm.setPosition(1);
        rightArm.setPosition(0);
        Thread.sleep(500);
        
        //Move Back
        start_angle = gyroCalc.currentAngle;
        enc_diff = drive_straight(-1, start_angle, end_angle, gyroCalc);
        end_angle = gyroCalc.currentAngle;
        Forward(0,0);
    }
    
    public void PulleyLift (double power, long time) throws InterruptedException
    {
        glyph_pulley.setPower(power);
        Thread.sleep(time);
        glyph_pulley.setPower(0);
    }
    
    @Override
    public void runOpMode() throws InterruptedException 
    {
        Init();
        GyroCalculations gyroCalc = new GyroCalculations(hardwareMap);        //gyro?
        waitForStart();
        relicTrackables.activate();
        // Start the timer
        Timer time = new Timer();
        time.scheduleAtFixedRate(gyroCalc, 0, 100);
        
        // Keep jewel arm away from pulley
        jewelArm.setPosition(0.7);
        jewelTurn.setPosition(0.5);
        Thread.sleep(500);
        
        // Grab Block
        leftArm.setPosition(0.35);
        rightArm.setPosition(0.45);
        Thread.sleep(1000);
        PulleyLift(0.5, 1000);
        Thread.sleep(1000);
        glyph_pulley.setPower(0.05);
        
        // Lower arm and wait for stabilization
        jewelArm.setPosition(0);
        jewelTurn.setPosition(0.55);
        Thread.sleep(1000);
        
        
        int RedVal = 0;
        int BlueVal = 0;
        for (int i = 0; i < 10; i++)
        {
            RedVal += colorSensor.red();
            BlueVal += colorSensor.blue();
        }
        
        RedVal /= 10;
        BlueVal /= 10;
        telemetry.addData("Color Sensor Blue Value", BlueVal);
        telemetry.addData("Color Sensor Red Value", RedVal);
        telemetry.update();
        
        boolean CurrentColor = true; //Blue = False, Red = True
        
        boolean ColorModeBlue = BlueVal > RedVal;
        if (CurrentColor == ColorModeBlue) 
        { 
            jewelTurn.setPosition(0);
            Thread.sleep(1000);
        }else
        {
            jewelTurn.setPosition(1);
            Thread.sleep(1000);
        }
        AutonPath(gyroCalc);
        
        time.cancel();
        time.purge();
    }
}