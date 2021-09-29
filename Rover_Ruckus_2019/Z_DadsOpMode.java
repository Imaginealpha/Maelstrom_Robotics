package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
//CAPITAL IN CONFIG, LOWERCASE IN VARIABLES
public class Z_DadsOpMode extends LinearOpMode {
    
    // private DcMotor left, right, arm, rollers;
    private Servo light;
    // private CRServo slide;
    
    @Override
    public void runOpMode() {
        
        //Get from hub
        // left = hardwareMap.get(DcMotor.class, "Left");
        // right = hardwareMap.get(DcMotor.class, "Right");
        // arm = hardwareMap.get(DcMotor.class, "Arm");
        // rollers = hardwareMap.get(DcMotor.class, "Rollers");
        light = hardwareMap.get(Servo.class, "Light");
        // slide = hardwareMap.get(CRServo.class, "Slide");
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            // Directions for wheels
            // left.setDirection(DcMotor.Direction.REVERSE);
            // right.setDirection(DcMotor.Direction.FORWARD);
            
            // left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            //Power control for DC motors
            // left.setPower(gamepad1.right_stick_y);
            // right.setPower(gamepad1.left_stick_y);
            // arm.setPower(0.4*gamepad2.right_stick_y);
            // rollers.setPower(gamepad2.left_stick_y);
            
            light.setPosition(0.7295);
                
            // slide.setPower(0.8*gamepad2.right_stick_x);

            //Telemetries
            // telemetry.addData("Left Power", left.getPower());
            // telemetry.addData("Right Power", right.getPower());
            // telemetry.addData("DC Arm Power", arm.getPower());
            // telemetry.addData("Roller Power", rollers.getPower());
            // telemetry.addData("Servo Slide Power", slide.getPower());
            telemetry.addData("Light Position", light.getPosition());
            telemetry.update();
          
            idle();
        }
    }
}
