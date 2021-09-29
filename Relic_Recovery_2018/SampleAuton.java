package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous
public class SampleAuton extends LinearOpMode {
    private DcMotor MyFirstDCMotor;
    private Servo MyFirstServo;
    
    // Defines the motor. Below is motor initilization.
    
    private void Init() {
        MyFirstDCMotor = hardwareMap.get(DcMotor.class, "FrontLeft");
        MyFirstServo = hardwareMap.get(Servo.class, "JewelTurn");
        MyFirstDCMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    // Exception to normal code. Initializes code and moves the motor.
    @Override
    public void runOpMode() throws InterruptedException {
        Init();
        waitForStart();
        MyFirstServo.setPosition(0); // 0 back view moves right
        //MoveMotorTime();
    }
    
    //Function to move the motor at half power for 1 second.
    public void MoveMotorTime() throws InterruptedException {
        MyFirstDCMotor.setPower(0.5);
        Thread.sleep(1000);
        MyFirstDCMotor.setPower(0);
    }
}