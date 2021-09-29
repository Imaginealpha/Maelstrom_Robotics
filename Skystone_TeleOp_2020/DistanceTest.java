package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;

@TeleOp

public class DistanceTest extends LinearOpMode {
    private DistanceSensor dist1;
    private DistanceSensor dist2;

    public void runOpMode() {
        
        dist1 = hardwareMap.get(DistanceSensor.class, "Distance 1");
        dist2 = hardwareMap.get(DistanceSensor.class, "Distance 2");
        
        waitForStart();
        
        while(opModeIsActive()) {
            telemetry.addData("Distance 1", dist1.getDistance(DistanceUnit.INCH));
            telemetry.addData("Distance 2", dist2.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}