package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class TensorFlow {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AcCRpBP/////AAABmW3pUDrNkE5SoqCszdTr+FgctocdfJq2xWHQbO/m7m8GHY6LT3qaL/Nz+vfOwclZ+fGVhL7bRXJswP8bPEv0EBMKcbuAjZT3Y6z+spndhuEawOJnjj9vATOn8w50SxuxxHa2kDMsdB2BwxbVFhZld+hAGeGvFuORLXOH/R2sQ8QqWU3j/5e8H77XghS4uUSgeYgA8k4nINRrfcMRT8nAodUefhRVf/ji2C+wUIC9dofQO1g5r2kz/yuGjJOSHA6gRg6OS+UabTB1Y0IMRxbEHAWdQCSBHl+eVHCTnOaJaJMris0HeIoRdRJWArN6Eky/4fy9FIN9LqJFHU/0NUWJITXaaTsMdC9m/Q5OWj6iWv0p";

    private static VuforiaLocalizer vuforia;
    private static TFObjectDetector tfod;
    public static boolean left = false, right = false, center = false, running = false;
    static HardwareMap hardwareMap;

    public static void detect() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        //initVuforia();
        
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }

        if (running) {
            // Activate Tensor Flow Object Detection.
            if (tfod != null) {
                tfod.activate();
            }

            while (running) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      //telemetry.addData("# Object Detected", updatedRecognitions.size());
                      if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                          if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                          } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                          } else {
                            silverMineral2X = (int) recognition.getLeft();
                          }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                          if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            //telemetry.addData("Gold Mineral Position", "Left");
                            left = true;
                          } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            //telemetry.addData("Gold Mineral Position", "Right");
                            right = true;
                          } else {
                            //telemetry.addData("Gold Mineral Position", "Center");
                            center = true;
                          }
                        }
                      }
                      //telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public static void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public static void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}