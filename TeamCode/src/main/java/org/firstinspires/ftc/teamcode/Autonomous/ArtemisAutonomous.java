/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * This is the Autonomous package
 * **/
package org.firstinspires.ftc.teamcode.Autonomous;

/**
 * Imports OpMode class and the Autonomous declaration and Elapsed Time
 * **/
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Imports the list collection from Java
 * **/
import java.util.List;

/**
 * Imports Vuforia libraries
 * **/
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Imports the tensorflow lite object detector and object recognition
 * **/
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * Imports the Artemis Hardware Map Class which initializes all the hardware once the user presses the INIT button
 * **/
import org.firstinspires.ftc.teamcode.HardwareMap.ArtemisHardwareMap;

/**
 * The name attribute specifies the OpMode name on the robot controller and group attribute specifies the group
 * in which that OpMode is located
 * **/
/**
 * This class uses Tensorflow lite along with the webcam to dynamically choose wheter to score points via wobble goal
 * or shooting rings all on its own
 * **/
@Autonomous(name = "Artemis Autonomous")
public class ArtemisAutonomous extends LinearOpMode {

    /**
     * The hardware map initialization object which initializes all our motors and servos
     * **/
    ArtemisHardwareMap hardwareMapInitialize = new ArtemisHardwareMap();

    /**
     * This stores our instance of the Vuforia localization engine and the license key.
     */
    private static final String VUFORIA_KEY = "AZFZpQv/////AAABma3flO1LSErYoIle7LztDPNRoW1dHp8UeguAk39po+KNco8nBQHysaMDKgzW/BH4Ue3+xmBKFZGWOesaq1FWHBHBpX3v4xlIImr1jMgxvbMvripQmY6vApS6VM3KkX1zkJ/pjj0iZ0BPExCxFC3aEY/GdRhb6QVtsmQ156Un7by4Awrqhub1Hwu5Ve+tBaapU8jaEuxjGU3AtURKMvDibswkbdbMG4d8QqCKf2Eh4tXbeWds1ox6wdolhkQrZTdFuITJc/nW9bM0nh95hQPRBRA5hQl6KWRyCCcRTvfbVggppr5MFMGICuc/TxXnYknRYBjVH9jRWiVcJrOPmad0qTOn8/e9ZrtD8ofS9sW51mTO";
    private VuforiaLocalizer vuforia;

    /**
     * This stores our instance of the TensorFlow Object Detection engine and the corresponding ring variables.
     */
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private TFObjectDetector tfod;
    private int numberOfRings = 0;
    private ElapsedTime runtime = new ElapsedTime();

    /**
     * This method initializes hardware and logs it if it was successful
     * **/
    public void initializeHardware(){
        telemetry.addData("Robot Initialized Successfully in Autonomous", " Wait for Hardware to Initialize");
        telemetry.update();
        hardwareMapInitialize.init(hardwareMap);
        telemetry.addData("Robot Hardware Initialized Successfully in Autonomous", "Press Play to Start");
        telemetry.update();
    }

    /**
     * This method initializes the tensorflow software and the Webcam to initialize Vuforia
     * **/
    public void initializeTensorVuforia(){
        initVuforia();
        initTfod();
        if(tfod != null){
            tfod.activate();
        }
        telemetry.addData("Tensorflow and Vuforia ", "Initialized");
        telemetry.update();
        while(!opModeIsActive()){
            checkRings();
        }
        telemetry.addData("Number of Rings Detected",numberOfRings);
        telemetry.update();
    }

    /**
     * This method logs that the robot was stopped and shuts down tesnorflow
     * **/
    public void stopRobot(){
        telemetry.addData("Robot Status Autonomous: ", "Has Stopped");
        telemetry.update();
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Based on the number of rings the robot will call its respective method
     * **/
    @Override
    public void runOpMode() throws InterruptedException {

        /**
         * Initialize Hardware functions and Software functions called
         * **/
        initializeHardware();
        initializeTensorVuforia();

        //runs the actual opmode code here
        waitForStart();
        telemetry.addData("Robot Status Autonomous: ", "Is in Play Mode");
        telemetry.update();

        //1. Move forward till half of field 0.2s
        hardwareMapInitialize.autonomousMotorMove(0.25);
        runtime.reset();
        while( opModeIsActive() && runtime.seconds() < 0.2){
            telemetry.addData("Moving Robot ","Forwards");
            telemetry.update();
        }
        sleep(1000);

        //2. Shoot all 3 rings
        hardwareMapInitialize.autonomousMotorMove(0.0);
        hardwareMapInitialize.autonomousMotorShoot(0.85);
        runtime.reset();
        while( opModeIsActive() && runtime.seconds()<5){
            telemetry.addData("Shooting Rings ","Mid Goal");
            telemetry.update();
        }
        sleep(1000);

        //3 Move robot right
        hardwareMapInitialize.autonomousMotorShoot(0);
        hardwareMapInitialize.autonomousMotorTurn(0.5,0);
        runtime.reset();
        while (runtime.seconds()<0.5){
            telemetry.addData("Moving Robot ","Right");
            telemetry.update();
        }
        sleep(1000);

        //4. Place Wobble Goal
        hardwareMapInitialize.autonomousMotorTurn(0,0);
        hardwareMapInitialize.autonomousServoHandle(true);
        runtime.reset();
        while (runtime.seconds()<2){
            telemetry.addData("Placing Wobble ","Right");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);

        /**
         * Calls stop function when Opmode goes from active to not active
         * **/
        stopRobot();
    }


    /**
     * Robot will either go place the wobble first(0,1 rings) or shoot the rings first(4 rings)
     * **/

    /**
     * This method is called when Tensorflow detects no rings on the starter stack
     * **/
    public void zeroRings(){

    }

    /**
     * This method is called when Tensorflow detects 1 ring on the starter stack
     * **/
    public void oneRings(){

    }

    /**
     * This method is called when Tensorflow detects 4 rings on the starter stack
     * **/
    public void fourRings(){

    }

    /**
     * Initialize the Vuforia localization engine along with the webcam
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine with its respective assets and a way to display 0,1, or 4 rings detected
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    //Tensorflow and vuforia will use the camera to detect how many rings there are
    public void checkRings(){
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                telemetry.update();
                if(updatedRecognitions.size() == 0){
                    // empty list.  no objects recognized.
                    telemetry.addData("TFOD", "No items detected.");
                    telemetry.update();
                    numberOfRings = 0;
                }else{
                    // list is not empty.
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        telemetry.update();
                        // check label to see which target zone to go after.
                        if (recognition.getLabel().equals("Single")) {
                            numberOfRings = 1;
                        } else if (recognition.getLabel().equals("Quad")) {
                            numberOfRings = 4;
                        } else {
                            numberOfRings= 0;
                        }
                    }
                }
            }
        }
    }
}
