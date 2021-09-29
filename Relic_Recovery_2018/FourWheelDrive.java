package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
//CAPITAL IN CONFIG, LOWERCASE IN VARIABLES
public class FourWheelDrive extends LinearOpMode {
    
    private DcMotor left = null;
    private DcMotor right = null;
    private DcMotor rollers = null;
    private DcMotor slide = null;
    //private Servo hook= null;
    
    @Override
    public void runOpMode() {
        
        //Get from hub
        left = hardwareMap.get(DcMotor.class, "Left");
        right = hardwareMap.get(DcMotor.class, "Right");
        rollers = hardwareMap.get(DcMotor.class, "Rollers/Left");
        slide = hardwareMap.get(DcMotor.class, "Slide/Right");
        //hook = hardwareMap.get(Servo.class, "Hook");
        
        // Set motors to zero power braking
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        //Target powers
        double tgtPowerLeft = 0;
        double tgtPowerRight = 0;
        double tgtPowerRollers = 0;
        double tgtPowerSlide = 0;
        
        //hook.setPosition(0);
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            //Directions for wheels
            left.setDirection(DcMotor.Direction.REVERSE);
            right.setDirection(DcMotor.Direction.REVERSE);
            rollers.setDirection(DcMotor.Direction.REVERSE);
            
            //Gamepad directions
            tgtPowerRight = gamepad1.right_stick_y;
            tgtPowerLeft = gamepad1.left_stick_y;
            tgtPowerRollers = gamepad1.left_stick_y;
            tgtPowerSlide = gamepad1.right_stick_y;
            
            //Power control for wheels
            left.setPower(-0.3*tgtPowerLeft);
            right.setPower(0.3*tgtPowerRight);
            
            rollers.setPower(-0.8*tgtPowerRollers);
            slide.setPower(-0.3*tgtPowerSlide);
            
            // if (gamepad1.b) {
            //     hook.setPosition(0.5);
            // } else if (gamepad1.dpad_left) {
            //     hook.setPosition(0);
            // } else if (gamepad1.x) {
            //     hook.setPosition(1);
            // }
            
            if (gamepad1.y) {
                // brake
            }
            
            //Telemetries
            telemetry.addData("Left Power", left.getPower());
            telemetry.addData("Right Power", right.getPower());
            telemetry.addData("Roller Power", rollers.getPower());
            telemetry.addData("Slide Power", slide.getPower());
            //telemetry.addData("Hook Position", hook.getPosition());
            telemetry.update();
          
            idle();
        }
        }
    
    }
