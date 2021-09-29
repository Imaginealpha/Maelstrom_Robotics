package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.util.Hardware;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

public class TensorFlow {
    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    public static final String VUFORIA_KEY = "AcCRpBP/////AAABmW3pUDrNkE5SoqCszdTr+FgctocdfJq2xWHQbO/m7m8GHY6LT3qaL/Nz+vfOwclZ+fGVhL7bRXJswP8bPEv0EBMKcbuAjZT3Y6z+spndhuEawOJnjj9vATOn8w50SxuxxHa2kDMsdB2BwxbVFhZld+hAGeGvFuORLXOH/R2sQ8QqWU3j/5e8H77XghS4uUSgeYgA8k4nINRrfcMRT8nAodUefhRVf/ji2C+wUIC9dofQO1g5r2kz/yuGjJOSHA6gRg6OS+UabTB1Y0IMRxbEHAWdQCSBHl+eVHCTnOaJaJMris0HeIoRdRJWArN6Eky/4fy9FIN9LqJFHU/0NUWJITXaaTsMdC9m/Q5OWj6iWv0p";

    public static VuforiaLocalizer vuforia;
    public static TFObjectDetector tfod;
    public static boolean left = false, right = false, center = false;
    public static int size = 0;
    public static int goldMineralX = -1;
    public static int silverMineral1X = -1;
    public static int silverMineral2X = -1;
    public static int cnt_left, cnt_right, cnt_center;

    public static void detect() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        //initVuforia();
        
        // if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
        //     initTfod(hardwareMap);
        // }

        assert tfod != null;
        
            // Activate Tensor Flow Object Detection.
            if (tfod != null) {
                tfod.activate();
            }

                int count_left = 0;
                int count_right = 0;
                int count_center = 0;
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    for (int i = 0; i < 2; i++) {
                      goldMineralX = -1;
                      silverMineral1X = -1;
                      silverMineral2X =-1;
                      List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                      if (updatedRecognitions != null) {
                        //telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 2) {
                          size = updatedRecognitions.size();
                          
                          for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                              goldMineralX = (int) recognition.getLeft();
                              // telemetry.addData("Gold Position", goldMineralX);
                            } else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL) && silverMineral1X == -1) {
                              silverMineral1X = (int) recognition.getLeft();
                              // telemetry.addData("Silver 1 Position", silverMineral1X);
                            } else if (silverMineral2X == -1) {
                              silverMineral2X = (int) recognition.getLeft();
                              // telemetry.addData("Silver 2 Position", silverMineral2X);
                            }
                          }
                          
                          if (goldMineralX != -1 && silverMineral1X != -1)
                            silverMineral2X = -1;
                              
                          // Positions (to the robot)
                          if (goldMineralX == -1)
                            count_right++;
                          else if (goldMineralX < silverMineral1X)
                            count_left++;
                          else if (goldMineralX > silverMineral1X)
                            count_center++;
                        }
                      }
                      Thread.sleep(600);
                    }
                    
                    // cnt_left = count_left;
                    // cnt_right = count_right;
                    // cnt_center = count_center;
                    
                    if (count_left > count_right && count_left > count_center)
                      left = true;
                    else if (count_right > count_center && count_right > count_left)
                      right = true;
                    else if (count_center > count_right && count_center > count_left)
                      center = true;
        }

        // VuforiaBehaviour.Instance.enabled = false;
    }

    public static void initVuforia(HardwareMap hardwareMap) {
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

    public static void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    
    public static int getSize() {
      return size;
    }
    
    public static int getPos(String pos) {
      if (pos.equals("Gold"))
        return goldMineralX;
      else if (pos.equals("Silver1"))
        return silverMineral1X;
      else if (pos.equals("Silver2"))
        return silverMineral2X;
      else
        return 772190;
    }
    
    // public static int getCount(String pos) {
    //   if (pos.equals("Left"))
    //     return cnt_left;
    //   else if (pos.equals("Right"))
    //     return cnt_right;
    //   else if (pos.equals("Center"))
    //     return cnt_center;
    //   else
    //     return 772190;
    // }
}