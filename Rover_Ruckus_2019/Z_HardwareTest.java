package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class Z_HardwareTest extends LinearOpMode {
    private DistanceSensor prox;

    @Override
    public void runOpMode() {
        prox = hardwareMap.get(DistanceSensor.class, "Prox");
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            if (prox.getDistance(DistanceUnit.INCH) < 4)
                telemetry.addData("WARNING", "STOP");
            else if (prox.getDistance(DistanceUnit.INCH) != Double.NaN)
                telemetry.addData("OPERATION", "HALTED");
            else
                telemetry.addData("OPERATION", "SMOOTH");
            
            telemetry.addData("Distance (in)", prox.getDistance(DistanceUnit.INCH));
            telemetry.update();
            
            idle();
        }
    }
}
