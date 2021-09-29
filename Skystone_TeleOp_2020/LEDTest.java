package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Gyroscope;

@TeleOp
@Disabled
public class LEDTest extends LinearOpMode {
    private RevBlinkinLedDriver light;
    private TouchSensor touch, touch1, touch2;
    private RevBlinkinLedDriver.BlinkinPattern pattern;

    @Override
    public void runOpMode() throws InterruptedException {
        light = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
        touch = hardwareMap.get(TouchSensor.class, "Touch");
        touch1 = hardwareMap.get(TouchSensor.class, "Wheel Touch 1");
        touch2 = hardwareMap.get(TouchSensor.class, "Wheel Touch 2");
        
        pattern = RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE;
        
        light.setPattern(pattern);
        
        waitForStart();
        
        while (opModeIsActive()) {
            telemetry.addData("touch", touch.isPressed());
            telemetry.addData("touch1", touch1.isPressed());
            telemetry.addData("touch2", touch2.isPressed());
            if (touch.isPressed()) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            } else if (touch1.isPressed() && touch2.isPressed()) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
            } else if (touch1.isPressed() || touch2.isPressed()) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
            } else {
                pattern = RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE;
            }
            light.setPattern(pattern);
            telemetry.addData("light color", pattern);
            telemetry.update();
        }
    }
}