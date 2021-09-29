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
public class IndividualWheelTest extends LinearOpMode {

    // teleop code whatever the heck we decide to do
    private DcMotor rightb;
    
    @Override
    public void runOpMode() throws InterruptedException {
        
        rightb = hardwareMap.get(DcMotor.class, "Back Right");
        
        telemetry.addData(">", "Ready to start");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            rightb.setPower(gamepad1.right_stick_y);
            
            telemetry.addData("Back Right Power", rightb.getPower());
            telemetry.update();
        }
    };
}