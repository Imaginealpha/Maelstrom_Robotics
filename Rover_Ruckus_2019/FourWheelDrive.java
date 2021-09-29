package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
//CAPITAL IN CONFIG, LOWERCASE IN VARIABLES
public class FourWheelDrive extends LinearOpMode {
    
    private DcMotor leftf, rightf, leftb, rightb, slide, collect, lift, extend;
    private Servo latchb, latcht, stopper, light;
    private CRServo rollers;
    private DistanceSensor prox;
    
    @Override
    public void runOpMode() {
        
        //Get from hub
        leftf = hardwareMap.get(DcMotor.class, "LeftF");
        rightf = hardwareMap.get(DcMotor.class, "RightF");
        leftb = hardwareMap.get(DcMotor.class, "LeftB");
        rightb = hardwareMap.get(DcMotor.class, "RightB");
        slide = hardwareMap.get(DcMotor.class, "Slide");
        collect = hardwareMap.get(DcMotor.class, "Collection");
        lift = hardwareMap.get(DcMotor.class, "Lift");
        extend = hardwareMap.get(DcMotor.class, "Extend");
        latchb = hardwareMap.get(Servo.class, "LatchB");
        latcht = hardwareMap.get(Servo.class, "LatchT");
        stopper = hardwareMap.get(Servo.class, "Stopper");
        // light = hardwareMap.get(Servo.class, "Light");
        rollers = hardwareMap.get(CRServo.class, "Rollers");
        prox = hardwareMap.get(DistanceSensor.class, "FrontProx");
        
        // Set motors to zero power braking
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collect.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        
        //Target powers
        double tgtPowerLeft = 0;
        double tgtPowerRight = 0;
        double tgtPowerSlide = 0;
        double tgtPowerCollect = 0;
        double tgtPowerLift = 0;
        double tgtPowerExt = 0;
        
        // light.setPosition(0.7095);
        
        latchb.setPosition(0);
        latcht.setPosition(0.1);
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            // light.setPosition(0.7295);
            
            //Directions for wheels
            leftf.setDirection(DcMotor.Direction.FORWARD);
            leftb.setDirection(DcMotor.Direction.FORWARD);
            rightf.setDirection(DcMotor.Direction.FORWARD);
            rightb.setDirection(DcMotor.Direction.REVERSE);
            slide.setDirection(DcMotor.Direction.REVERSE);
            collect.setDirection(DcMotor.Direction.FORWARD);
            
            //Gamepad directions
            tgtPowerSlide = gamepad2.right_stick_x;
            tgtPowerCollect = gamepad2.right_stick_y;
            tgtPowerLift = gamepad2.left_stick_y;
            tgtPowerExt = gamepad2.left_stick_x;

            int count = 0;

            if (gamepad1.right_bumper) {
                count++;
            }
            if (count % 2 == 0) {
                tgtPowerLeft = gamepad1.left_stick_y;
                tgtPowerRight = gamepad1.right_stick_y;
            } else {
                tgtPowerLeft = 0.3*gamepad1.left_stick_y;
                tgtPowerRight = 0.3*gamepad1.right_stick_y;
            }

            //Power control
            leftf.setPower(tgtPowerLeft);
            rightf.setPower(tgtPowerRight);
            leftb.setPower(tgtPowerLeft);
            rightb.setPower(tgtPowerRight);
            collect.setPower(0.8*tgtPowerCollect);
            lift.setPower(tgtPowerLift);
            slide.setPower(tgtPowerSlide);
            extend.setPower(tgtPowerExt);
            
            int cntLeft = 0, cntRight = 0;
            
            if (gamepad2.left_bumper)
                rollers.setPower(-1);
            if (gamepad2.right_bumper)
                rollers.setPower(1);
            if (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0)
                rollers.setPower(0);
            
            
            if (gamepad2.y)
                latchb.setPosition(1);
            else if (gamepad2.dpad_up)
                latchb.setPosition(0);
                
            if (gamepad2.b)
                latcht.setPosition(1);      // closed
            else if (gamepad2.dpad_left)
                latcht.setPosition(0.07);    // opened
                
            if (gamepad1.right_bumper)
                stopper.setPosition(0);
            else
                stopper.setPosition(1);
            
            //Telemetries
            
            if (prox.getDistance(DistanceUnit.INCH) < 4)
                telemetry.addData("WARNING", "STOP");
            else if (prox.getDistance(DistanceUnit.INCH) != Double.NaN)
                telemetry.addData("OPERATION", "HALTED");
            else
                telemetry.addData("OPERATION", "SMOOTH");
            
            telemetry.addData("Distance (in)", prox.getDistance(DistanceUnit.INCH));
            telemetry.addData("Left Power", leftf.getPower());
            telemetry.addData("Right Power", rightf.getPower());
            telemetry.addData("Slide Power", slide.getPower());
            telemetry.addData("Collection Power", collect.getPower());
            telemetry.addData("Collection Extension Power", extend.getPower());
            telemetry.addData("Roller Power", rollers.getPower());
            telemetry.addData("Lift Power", lift.getPower());
            telemetry.addData("Top Latch Position", latcht.getPosition());
            telemetry.addData("Bottom Latch Position", latchb.getPosition());
            telemetry.addData("Stopper Position", stopper.getPosition());
            telemetry.update();
          
            idle();
        }
    }
}