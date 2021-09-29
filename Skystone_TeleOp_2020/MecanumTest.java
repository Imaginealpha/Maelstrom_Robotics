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
public class MecanumTest extends LinearOpMode {

    // teleop code whatever the heck we decide to do
    private DcMotor leftf, leftb, rightf, rightb;
    
    double pwr_ctrl = 1;
    
    @Override
    public void runOpMode() {
        
        leftf = hardwareMap.get(DcMotor.class, "Front Left");
        leftb = hardwareMap.get(DcMotor.class, "Back Left");
        rightf = hardwareMap.get(DcMotor.class, "Front Right");
        rightb = hardwareMap.get(DcMotor.class, "Back Right");
        
        leftf.setDirection(DcMotor.Direction.REVERSE);
        // leftb.setDirection(DcMotor.Direction.REVERSE);
        // rightf.setDirection(DcMotor.Direction.REVERSE);
        // rightb.setDirection(DcMotor.Direction.REVERSE);
        
        telemetry.addData(">", "Ready to start");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            double x = gamepad1.left_stick_y * pwr_ctrl;    // drive
            double r = gamepad1.right_stick_y * pwr_ctrl;   // rotate
            double y = 0;                                   // strafe
            
            if (gamepad1.left_trigger > 0)
                y = gamepad1.left_trigger * pwr_ctrl;
            else if (gamepad1.right_trigger > 0)
                y = -gamepad1.right_trigger * pwr_ctrl;
            
            double flpwr = x + y + r;
            double frpwr = x - y - r;
            double blpwr = x - y + r;
            double brpwr = x + y - r;
            
            leftf.setPower(flpwr);
            leftb.setPower(blpwr);
            rightf.setPower(frpwr);
            rightb.setPower(brpwr);

            //Telemetries
            telemetry.addData("Front Left Power", leftf.getPower());
            telemetry.addData("Back Left Power", leftb.getPower());
            telemetry.addData("Front Right Power", rightf.getPower());
            telemetry.addData("Back Right Power", rightb.getPower());
            telemetry.update();
        }
    };
}