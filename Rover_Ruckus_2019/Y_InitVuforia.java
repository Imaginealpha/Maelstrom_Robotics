package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;

public class Y_InitVuforia {
    
    HardwareMap hardwareMap;
    
    static final String VUFORIA_KEY = "Ab0ZZvP/////AAABmTL2SBME2U9Au3hEMvcou6dhA3G1jcHQ9TXjLimKxd3QMrb7ky7DA+I/CGR19kSrodIcFSQMrCNkKzyaaV88GDY+3vJVYfNI/ypHBp5unYkoeOk3DTirR1A9X5wNjKrxFGMvU1bkFUysvFTk7+xVPTkBjOXPMr6ZrJ1G40Uc7hh7/sa86kGwsbfEfauI4OOhR7DDwPavAHTSIi1cnoeDnORPMRcJW9DVawZDvNTjfYtQUuEw6zzEWkmdO9qj72KFotuMvMk0hJ+FLm+MvA/40bmv5LpmsgRW1U8/w380Vs8e4n7ZxqSq+k6wgcKXZkR4g1+ruDmDdZhpic3iizee0tRKW5lRvJV4kRmvW9Hx2eGL";

    // ImageTarget trackables use mm to specify their dimensions
    // Constants and conversions
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // width of field (center pt to outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // height of center of target image above floor

    // Camera select
    static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    
    List<VuforiaTrackable> allTrackables;
    

    public static List<VuforiaTrackable> Init(HardwareMap hardwareMap, VuforiaLocalizer.Parameters parameters, VuforiaLocalizer vuforia) throws InterruptedException {
        
        // Load data sets for trackable objects
        VuforiaTrackables targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");
        
        // Rover
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
    
        // Moon
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        
        // Mars
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        
        // Space
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        
        // Gather all trackable objects together into one list
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);
        
        // Target positions:
            /* 
             *  Axis directions (see chart):
             *
             *  If you are standing in the Red Alliance Station looking towards the center of the field,
             *     - The X axis runs from your left to the right. (positive from the center to the right)
             *     - The Y axis runs from the Red Alliance Station towards the other side of the field
             *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
             *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
             *
             *  All targets are first assumed to be in the center, on the floor, face up.
             *  We must set the position of the targets in the following code.
             */
             
//-----------------------------------------------------------------------------------------------------------------------------------------------------------
    
             /**
             * To place the BlueRover target in the middle of the blue perimeter wall:
             * - Rotate 90 around X axis to flip it upright.
             * - Translate along Y axis to blue perimeter wall.
             */
            OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                    .translation(0, mmFTCFieldWidth, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
            blueRover.setLocation(blueRoverLocationOnField);
            
            /**
             * To place the RedFootprint target in the middle of the red perimeter wall:
             * - Rotate 90 around X axis to flip it upright.
             * - Rotate 180 around Z axis so it is flat against the red perimeter wall.
             * - Translate along negative Y axis to red perimeter wall.
             */
            OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                    .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
            redFootprint.setLocation(redFootprintLocationOnField);
            
            /**
             * To place the FrontCraters target in the middle of the front perimeter wall:
             * - Rotate 90 around X axis to flip it upright.
             * - Rotate it 90 around Z axis so the image is flat against the front wall.
             * - Translate along negative X axis to front perimeter wall.
             */
            OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                    .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
            frontCraters.setLocation(frontCratersLocationOnField);
            
            /**
             * To place the BackSpace target in the middle of the back perimeter wall:
             * - Rotate 90 around X axis to flip it upright.
             * - Rotate it -90 around Z axis so the image is flat against the back wall.
             * - Translate along X axis to the back perimeter wall.
             */
            OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                    .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
            backSpace.setLocation(backSpaceLocationOnField);

        
        /* Create a transformation matrix to position the phone relative to the field.
         *
         * Top of the camera MUST point to left side of robot!
         * Rotate camera to bring required camera forward. Current camera: REAR
         *
         * Place the camera position.
         * In this example, it is centered (left to right), but 110 mm forward from center and 200 mm above ground level.
         */
         
        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));
        
        // Let all the trackable listeners know where the phone is.
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }
        
        // Start tracking data sets
        targetsRoverRuckus.activate();
        
        return allTrackables;
    }
    
    public String VuMark(List<VuforiaTrackable> allTrackables) {
        
        // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            String name = "none";

            //RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    name = trackable.getName();
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
            
            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                //telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                //        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                
            }
            
            return name;
    }
}
