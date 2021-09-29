package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.vuforia.CameraDevice;

@TeleOp (name = "TensorFlow Test")
public class X_TensorFlowTest extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AcCRpBP/////AAABmW3pUDrNkE5SoqCszdTr+FgctocdfJq2xWHQbO/m7m8GHY6LT3qaL/Nz+vfOwclZ+fGVhL7bRXJswP8bPEv0EBMKcbuAjZT3Y6z+spndhuEawOJnjj9vATOn8w50SxuxxHa2kDMsdB2BwxbVFhZld+hAGeGvFuORLXOH/R2sQ8QqWU3j/5e8H77XghS4uUSgeYgA8k4nINRrfcMRT8nAodUefhRVf/ji2C+wUIC9dofQO1g5r2kz/yuGjJOSHA6gRg6OS+UabTB1Y0IMRxbEHAWdQCSBHl+eVHCTnOaJaJMris0HeIoRdRJWArN6Eky/4fy9FIN9LqJFHU/0NUWJITXaaTsMdC9m/Q5OWj6iWv0p";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
      initVuforia();

      if (ClassFactory.getInstance().canCreateTFObjectDetector())
          initTfod();
      else
          telemetry.addData("Sorry!", "This device is not compatible with TFOD");
      
      telemetry.addData(">", "Press Play to start tracking");
      telemetry.update();
      waitForStart();
      
      if (opModeIsActive()) {
        
        if (tfod != null)
            tfod.activate();

        while (opModeIsActive()) {
          CameraDevice.getInstance().setFlashTorchMode(true);
          
          if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
              telemetry.addData("# Object Detected", updatedRecognitions.size());
              if (updatedRecognitions.size() == 2) {
                int goldMineralX = -1;
                int silverMineral1X = -1;
                int silverMineral2X = -1;
                for (Recognition recognition : updatedRecognitions) {
                  if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                    goldMineralX = (int) recognition.getLeft();
                    telemetry.addData("Gold Position", goldMineralX);
                  } else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL) && silverMineral1X == -1) {
                    silverMineral1X = (int) recognition.getLeft();
                    telemetry.addData("Silver 1 Position", silverMineral1X);
                  } else if (silverMineral2X == -1) {
                    silverMineral2X = (int) recognition.getLeft();
                    telemetry.addData("Silver 2 Position", silverMineral2X);
                  }
                  telemetry.update();
                }
                
                if (goldMineralX != -1 && silverMineral1X != -1)
                  silverMineral2X = -1;
                
                if (silverMineral1X != -1 && silverMineral2X != -1)                 // DON'T FORGET TO TAKE OUT THIS ENTIRE IF STATEMENT AND UNCOMMENT THE ONE BELOW LATER
                  telemetry.addData("Gold Mineral Position", "Right");
                else if (goldMineralX < silverMineral1X)
                  telemetry.addData("Gold Mineral Position", "Left");
                else
                  telemetry.addData("Gold Mineral Position", "Center");
              
                
                  // leave this commented please
                // if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                //   if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                //     telemetry.addData("Gold Mineral Position", "Left");
                //   } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                //     telemetry.addData("Gold Mineral Position", "Right");
                //   } else {
                //     telemetry.addData("Gold Mineral Position", "Center");
                //   }
                // }
              }
              telemetry.update();
              
              Thread.sleep(10000);
            }
          }
        }
      }

        if (tfod != null)
          tfod.shutdown();
        
        CameraDevice.getInstance().setFlashTorchMode(false);

      }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
