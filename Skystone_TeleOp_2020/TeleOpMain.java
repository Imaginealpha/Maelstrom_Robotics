package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class TeleOpMain extends LinearOpMode {

    // teleop code whatever the heck we decide to do
    private DcMotor leftf, leftb, rightf, rightb, left_slide, right_slide, rollers, extend;
    private Servo grab, left_clamp, right_clamp;
    private TouchSensor touch, wheel_touch_1, wheel_touch_2;
    private RevBlinkinLedDriver light;
    private RevBlinkinLedDriver.BlinkinPattern pattern;
    private ElapsedTime timer;
    
    double pwr_ctrl_wheels = 1;
    double pwr_ctrl_arms = 1;
    
    @Override
    public void runOpMode() throws InterruptedException {
        
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
        light = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
        touch = hardwareMap.get(TouchSensor.class, "Touch");
        wheel_touch_1 = hardwareMap.get(TouchSensor.class, "Wheel Touch 1");
        wheel_touch_2 = hardwareMap.get(TouchSensor.class, "Wheel Touch 2");
        timer = new ElapsedTime();
        
        pattern = RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE;
        light.setPattern(pattern);
        
        left_clamp.setPosition(0);
        right_clamp.setPosition(1);
        
        telemetry.addData(">", "Ready to start");
        telemetry.update();
        
        waitForStart();
        
        timer.reset();
        
        while (opModeIsActive()) {
            
            if (gamepad2.left_bumper)
                timer.reset();
        
            // rightf.setDirection(DcMotor.Direction.REVERSE);
            // rightb.setDirection(DcMotor.Direction.REVERSE);
            leftf.setDirection(DcMotor.Direction.REVERSE);
            leftb.setDirection(DcMotor.Direction.REVERSE);
            right_slide.setDirection(DcMotor.Direction.REVERSE);
            // extend.setDirection(DcMotor.Direction.REVERSE);
            
            left_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            // Slow mode
            if (gamepad1.dpad_down)
                pwr_ctrl_wheels = 0.4;
            if (gamepad1.a)
                pwr_ctrl_wheels = 1;
            if (gamepad2.dpad_right)
                pwr_ctrl_arms = 0.5;
            if (gamepad2.x)
                pwr_ctrl_arms = 1;
            
            // Wheel controls
            double x = gamepad1.left_stick_y * pwr_ctrl_wheels;    // drive
            double r = gamepad1.right_stick_y * pwr_ctrl_wheels;   // rotate
            double y = 0;                                   // strafe
            
            if (gamepad1.left_trigger > 0)
                y = gamepad1.left_trigger * pwr_ctrl_wheels;
            else if (gamepad1.right_trigger > 0)
                y = -gamepad1.right_trigger * pwr_ctrl_wheels;
            
            double flpwr = x + y + r;
            double frpwr = x - y - r;
            double blpwr = x - y + r;
            double brpwr = x + y - r;
            
            leftf.setPower(flpwr);
            leftb.setPower(blpwr);
            rightf.setPower(frpwr);
            rightb.setPower(brpwr);
            
            // Slide controls
            left_slide.setPower(gamepad2.left_stick_y * pwr_ctrl_arms);
            right_slide.setPower(left_slide.getPower());
            
            // Grab controls
            if (gamepad2.b) {
                grab.setPosition(0.4);
            } else if (gamepad2.dpad_left) {
                grab.setPosition(1);
            }
            
            // Roller controls
            if (gamepad2.left_trigger > 0)
                rollers.setPower(-gamepad2.left_trigger * pwr_ctrl_arms);
            else if (gamepad2.right_trigger > 0)
                rollers.setPower(gamepad2.right_trigger * pwr_ctrl_arms);
            else
                rollers.setPower(0);
            
            // Extension control
            extend.setPower(gamepad2.right_stick_y * pwr_ctrl_arms);
            
            // Foundation clamp controls
            if (gamepad2.dpad_up) {         // down
                left_clamp.setPosition(1);
                right_clamp.setPosition(0);
            }
            else if (gamepad2.y) {          // up
                left_clamp.setPosition(0);
                right_clamp.setPosition(1);
            }
            
            // LED controls
            if (touch.isPressed()) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            } else if (wheel_touch_1.isPressed() && wheel_touch_2.isPressed()) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
            } else if (wheel_touch_1.isPressed() || wheel_touch_2.isPressed()) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
            } else if (timer.time() >= 90 && timer.time() <= 95) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
            } else if (timer.time() >= 110 && timer.time() <= 120) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET;
            } else {
                pattern = RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE;
            }
            light.setPattern(pattern);

            //Telemetries
            telemetry.addData("Front Left Power", leftf.getPower());
            telemetry.addData("Front Right Power", rightf.getPower());
            telemetry.addData("Back Left Power", leftb.getPower());
            telemetry.addData("Back Right Power", rightb.getPower());
            telemetry.addData("Left Slide Power", left_slide.getPower());
            telemetry.addData("Right Slide Power", right_slide.getPower());
            telemetry.addData("Roller Power", rollers.getPower());
            telemetry.addData("Extension Power", extend.getPower());
            telemetry.addData("Grab Position", grab.getPosition());
            telemetry.addData("Left Clamp Position", left_clamp.getPosition());
            telemetry.addData("Right Clamp Position", right_clamp.getPosition());
            telemetry.addData("Wheel Control", pwr_ctrl_wheels);
            telemetry.addData("Arm Control", pwr_ctrl_arms);
            telemetry.addData("Stone Touch Pressed", touch.isPressed());
            telemetry.addData("Wheel Touch 1 Pressed", wheel_touch_1.isPressed());
            telemetry.addData("Wheel Touch 2 Pressed", wheel_touch_2.isPressed());
            telemetry.addData("Light Color", pattern);
            telemetry.addData("Time Elapsed", timer.time());
            telemetry.update();
        }
    };
}