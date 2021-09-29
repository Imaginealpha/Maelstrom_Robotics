package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
@Disabled
public class ButtonTest extends LinearOpMode {
    private TouchSensor button;

    @Override
    public void runOpMode() {
        button = hardwareMap.get(TouchSensor.class, "Button");
        //button.setMode(DigitalChannel.Mode.INPUT);
        
        waitForStart();
        
        while (opModeIsActive()) {
            if (!button.isPressed())
                telemetry.addData(">", "Button not pressed.");
            else
                telemetry.addData(">", "Button pressed!");
            telemetry.update();
        }
    }
}