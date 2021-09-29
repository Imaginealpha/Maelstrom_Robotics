package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class DriveTrainTest extends LinearOpMode {

    // teleop code whatever the heck we decide to do
    private DcMotor leftf, leftb, rightf, rightb;
    
    double pwr_ctrl_wheels = 0.75;
    
    @Override
    public void runOpMode() throws InterruptedException {
        
        leftf = hardwareMap.get(DcMotor.class, "Front Left");
        rightf = hardwareMap.get(DcMotor.class, "Front Right");
        leftb = hardwareMap.get(DcMotor.class, "Back Left");
        rightb = hardwareMap.get(DcMotor.class, "Back Right");
        
        telemetry.addData(">", "Ready to start");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
        
            // rightf.setDirection(DcMotor.Direction.REVERSE);
            // rightb.setDirection(DcMotor.Direction.REVERSE);
            // leftf.setDirection(DcMotor.Direction.REVERSE);
            leftb.setDirection(DcMotor.Direction.REVERSE);
            
            rightf.setPower(gamepad1.right_stick_y);
            rightb.setPower(gamepad1.right_stick_y);
            leftf.setPower(gamepad1.left_stick_y);
            leftb.setPower(gamepad1.left_stick_y);

            //Telemetries
            telemetry.addData("Front Left Power", leftf.getPower());
            telemetry.addData("Front Right Power", rightf.getPower());
            telemetry.addData("Back Left Power", leftb.getPower());
            telemetry.addData("Back Right Power", rightb.getPower());
            telemetry.addData("Wheel Control", pwr_ctrl_wheels);
            telemetry.update();
        }
    };
}