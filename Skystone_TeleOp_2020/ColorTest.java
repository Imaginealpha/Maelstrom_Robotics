package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;

@TeleOp
public class ColorTest extends LinearOpMode {
    public ColorSensor color;
    
    @Override
    public void runOpMode() {
        color = hardwareMap.get(ColorSensor.class, "Color 2");
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            telemetry.addData("Red Value", color.red());
            telemetry.addData("Green Value", color.green());
            telemetry.addData("Blue Value", color.blue());
            telemetry.addData("Alpha Value", color.alpha());
            telemetry.update();
        }
    }
}