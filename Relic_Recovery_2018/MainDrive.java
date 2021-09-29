package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
//CAPITAL IN CONFIG, LOWERCASE IN VARIABLES
public class MainDrive extends LinearOpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor glyphPulley = null;
    private DcMotor middleWheel = null;
    private Servo leftArm = null;
    private Servo rightArm = null;
    private Servo jewelArm = null;
    private Servo jewelTurn = null;
    
    // Temp
    //private Servo hook = null;
    
    
    @Override
    public void runOpMode() {
        //Get from hub
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = hardwareMap.get(DcMotor.class, "BackRight");
        glyphPulley = hardwareMap.get(DcMotor.class, "GlyphPulley");
        middleWheel = hardwareMap.get(DcMotor.class, "MiddleWheel");
        leftArm = hardwareMap.get(Servo.class, "LeftArm");
        rightArm = hardwareMap.get(Servo.class, "RightArm");
        jewelArm = hardwareMap.get(Servo.class, "JewelArm");
        jewelTurn = hardwareMap.get(Servo.class, "JewelTurn");
        
        // Temp
        //hook = hardwareMap.get(Servo.class, "Hook");
        
        //Target powers
        double tgtPowerLeft = 0;
        double tgtPowerRight = 0;
        double tgtPowerBLeft = 0;
        double tgtPowerBRight = 0;
        double tgtPowerglyphPulley = 0;
        double tgtPowerMidWheel = 0;
        
        //Set Positions
        leftArm.setPosition(1);
        rightArm.setPosition(0);
        jewelArm.setPosition(0.65);
        jewelTurn.setPosition(0.5);
        
        //Wait
        waitForStart();
        
        
        while (opModeIsActive()) {
            //Directions for wheels
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            middleWheel.setDirection(DcMotor.Direction.FORWARD);
            
            
            //Gamepad directions for Wheels
            tgtPowerRight = gamepad1.right_stick_y;
            tgtPowerLeft = gamepad1.left_stick_y;
            tgtPowerBLeft = gamepad1.right_stick_y;
            tgtPowerBRight = gamepad1.left_stick_y;
            tgtPowerMidWheel = gamepad1.left_stick_x;
            
            //Power control for wheels
            frontLeft.setPower(-0.6*tgtPowerLeft);
            backRight.setPower(0.6*tgtPowerBLeft);
            backLeft.setPower(-0.6*tgtPowerBRight);
            frontRight.setPower(0.6*tgtPowerRight);
            middleWheel.setPower(-0.6*tgtPowerMidWheel);
            
            
            //Directions and Positions for Arm motors            
            glyphPulley.setDirection(DcMotor.Direction.FORWARD);
            
            boolean glyph = gamepad2.b;
            
            //Gamepad directions for Arm motors
            if (glyph == true){
                tgtPowerglyphPulley = 0.1;
                glyphPulley.setPower(tgtPowerglyphPulley);
            } else if (glyph == false){
                tgtPowerglyphPulley = gamepad2.right_stick_y;
                glyphPulley.setPower(-0.75*tgtPowerglyphPulley);
            }
            
            
            //tgtPowerRelicWrist = gamepad2.left_stick_y;
            //relicWrist.setPower(tgtPowerRelicWrist);
            
            // Cube Arm Directions
            if (gamepad2.left_bumper){
                leftArm.setPosition(1);
            } else if (gamepad2.right_bumper){
                rightArm.setPosition(0);
            } else if (gamepad2.dpad_up) {
                leftArm.setPosition(0.45);
            } else if (gamepad2.y) {
                rightArm.setPosition(0.55);
            }
            
            if (gamepad2.a){
                rightArm.setPosition(0.5);
            }
            
            
            // Temp
            // if (gamepad1.b) {
            //     hook.setPosition(0.5);
            // } else if (gamepad1.dpad_left) {
            //     hook.setPosition(0);
            // } else if (gamepad1.x) {
            //     hook.setPosition(1);
            // }
            
            
            
            //Telemetries
            telemetry.addData("Motor 0 Power", frontLeft.getPower());
            telemetry.addData("Motor 1 Power", frontRight.getPower());
            telemetry.addData("Motor 2 Power", backLeft.getPower());
            telemetry.addData("Motor 3 Power", backRight.getPower());
            telemetry.addData("Glyph Pulley Power", glyphPulley.getPower());
            telemetry.addData("Right Arm Position", rightArm.getPosition());
            telemetry.addData("Left Arm Position", leftArm.getPosition());
            telemetry.addData("Middle Wheel Power", middleWheel.getPower());
            //telemetry.addData("Relic Clamp Position", relicClamp.getPower());
            //telemetry.addData("Relic Wrist Position", relicWrist.getPower());
            telemetry.update();
          
            idle();
            
        }
        }
    
    }