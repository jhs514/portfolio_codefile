/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous

public class Challengers_Auto_right extends LinearOpMode {

    private Blinker control_Hub;
    private Blinker expansion_Hub_3;
    private HardwareDevice webcam_1;
    private IMU imu;
    private DcMotor l1motor;
    private DcMotor l2motor;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor r1motor;
    private DcMotor r2motor;
    private ColorSensor sensorColorRange;
    private Servo servo1;
    private Servo servo2;
    private Servo servo3;
    private Servo servo4;
    private Servo servo5;
    private TouchSensor touch;

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    //private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/apple_banana_grape2.tflite";


    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AXcBT1z/////AAABmcRZAGqEFEkoiV5XG5XsDX0Mw6HoMJe7Ge5up796G46nxGv+o/oaHfn/QtWjLmkFxxAlGkDMbyeJtGz5Ij50TdFZKIjifW18jHI7G/zhcDJv5LHt5qLTba5AJMnvypWHhDOpAwmVsKnUOyToYkAKp2wsdTX9/AgjEUfIF9Q6MucSjsthvbw7hkq+uvXgGg2ZbkBA/iPDopBA13Ymk1wjnuwZENxkQRmetdB9FjsteUorMd0pj+M2iZhKvt6UjIAmz4qqVUZhlqAOBvDz74yGT6V93/195KCDoW8HpYGwF06rPBRob9lJvvnniG5Ea9T1zTNAsD5VyRNxBuJiJUfd1o7XwyoAAZiL4UbMLn2CnF/E";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_3 = hardwareMap.get(Blinker.class, "Expansion Hub 3");
        webcam_1 = hardwareMap.get(HardwareDevice.class, "Webcam 1");
        imu = hardwareMap.get(IMU.class, "imu");
        l1motor = hardwareMap.get(DcMotor.class, "l1motor");
        l2motor = hardwareMap.get(DcMotor.class, "l2motor");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        r1motor = hardwareMap.get(DcMotor.class, "r1motor");
        r2motor = hardwareMap.get(DcMotor.class, "r2motor");
        sensorColorRange = hardwareMap.get(ColorSensor.class, "sensorColorRange");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");
        servo5 = hardwareMap.get(Servo.class, "servo5");
        touch = hardwareMap.get(TouchSensor.class, "Touch");
        
        r1motor.setDirection(DcMotor.Direction.REVERSE);
        r2motor.setDirection(DcMotor.Direction.REVERSE);
        l1motor.setDirection(DcMotor.Direction.FORWARD);
        l2motor.setDirection(DcMotor.Direction.FORWARD);
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        
        r1motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r1motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r2motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r2motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        l1motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l1motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        l2motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l2motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.FORWARD);
        servo3.setDirection(Servo.Direction.FORWARD);
        servo4.setDirection(Servo.Direction.FORWARD);
        servo5.setDirection(Servo.Direction.FORWARD);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }
        
        int arm1;
        double arm2;
        double arm3;
        double arm_dir;
        double arm_release;
        double arm_grab;
        
        motor1.setTargetPosition(50);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setPower(1);
        motor2.setTargetPosition(50);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setPower(1);
        while (motor1.isBusy() || motor2.isBusy()) {
        }
        
        servo1.setPosition(0.9);
        servo5.setPosition(0.9);
        servo2.setPosition(0.5);
        servo3.setPosition(0.03);
        servo4.setPosition(0.5);
        
        String isDetected = "None";
        double t = 0;

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            r1motor.setTargetPosition(500);
            r1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r1motor.setPower(1);
            l1motor.setTargetPosition(500);
            l1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l1motor.setPower(1);
            r2motor.setTargetPosition(500);
            r2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r2motor.setPower(1);
            l2motor.setTargetPosition(500);
            l2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l2motor.setPower(1);
            while (r1motor.isBusy() || r2motor.isBusy() || l1motor.isBusy() || l2motor.isBusy()) {
            }
            
            while (isDetected == "None") {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;
                            
                            if (recognition.getLabel().equals("1 Bolt")) {
                                isDetected = "1 Bolt";
                            } else if (recognition.getLabel().equals("2 Bulb")) {
                                isDetected = "2 Bulb";
                            } else if (recognition.getLabel().equals("3 Panel")) {
                                isDetected = "3 Panel";
                            }

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                        }
                        telemetry.update();
                    }
                }
            }
            
            int straight_tick = 2215;
            int turn_degree_90_tick = -1030;
            int back_tick = 0;
            int turn_degree_45_tick = -520;
            
            r1motor.setTargetPosition(straight_tick);
            r1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r1motor.setPower(1);
            l1motor.setTargetPosition(straight_tick);
            l1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l1motor.setPower(1);
            r2motor.setTargetPosition(straight_tick);
            r2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r2motor.setPower(1);
            l2motor.setTargetPosition(straight_tick);
            l2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l2motor.setPower(1);
            while (r1motor.isBusy() || r2motor.isBusy() || l1motor.isBusy() || l2motor.isBusy()) {
            }
            
            arm1 = 819;
            arm2 = 0.56;
            arm3 = 0.37;
            arm_dir = 0.69;
            
            int turn_exa = 15;
            
            motor1.setTargetPosition(arm1);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setPower(0.2);
            motor2.setTargetPosition(arm1);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setPower(0.2);
            
            t = getRuntime();
            while (getRuntime() - t < 0.1) {
            }
            
            servo1.setPosition(arm2);
            servo5.setPosition(arm2);
            servo2.setPosition(arm3);
            servo3.setPosition(arm_dir);
            
            r1motor.setTargetPosition(straight_tick - turn_degree_90_tick + turn_degree_45_tick + turn_exa);
            r1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r1motor.setPower(1);
            l1motor.setTargetPosition(straight_tick + turn_degree_90_tick - turn_degree_45_tick - turn_exa);
            l1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l1motor.setPower(1);
            r2motor.setTargetPosition(straight_tick - turn_degree_90_tick + turn_degree_45_tick + turn_exa);
            r2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r2motor.setPower(1);
            l2motor.setTargetPosition(straight_tick + turn_degree_90_tick - turn_degree_45_tick - turn_exa);
            l2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l2motor.setPower(1);
            
            while (r1motor.isBusy() || r2motor.isBusy() || l1motor.isBusy() || l2motor.isBusy()) {
            }
            
            //release
            
            t = getRuntime();
            while (getRuntime() - t < 1) {
            }
            
            servo4.setPosition(0.1);
            t = getRuntime();
            while (getRuntime() - t < 1) {
            }
            
            arm1 = 800;
            motor1.setTargetPosition(arm1);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setPower(0.5);
            motor2.setTargetPosition(arm1);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setPower(0.5);
            t = getRuntime();
            while (getRuntime() - t < 0.5) {
            }
            
            servo1.setPosition(0.9);
            servo5.setPosition(0.9);
            servo2.setPosition(0.5);
            servo3.setPosition(0.03);
            servo4.setPosition(0.1);
        
            motor1.setTargetPosition(50);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setPower(0.2);
            motor2.setTargetPosition(50);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setPower(0.2);
            
            r1motor.setTargetPosition(straight_tick - turn_degree_90_tick);
            r1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r1motor.setPower(1);
            l1motor.setTargetPosition(straight_tick + turn_degree_90_tick);
            l1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l1motor.setPower(1);
            r2motor.setTargetPosition(straight_tick - turn_degree_90_tick);
            r2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r2motor.setPower(1);
            l2motor.setTargetPosition(straight_tick + turn_degree_90_tick);
            l2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l2motor.setPower(1);
            while (r1motor.isBusy() || r2motor.isBusy() || l1motor.isBusy() || l2motor.isBusy()) {
            }
            /*
            // grab corn
            arm1 = 258;
            arm2 = 0.293;
            arm3 = 0.276;
            arm_dir = 0.03;
            arm_release = 0.1;
            arm_grab = 0.5;
            motor1.setTargetPosition(arm1);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setPower(1);
            motor2.setTargetPosition(arm1);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setPower(1);
            servo1.setPosition(arm2);
            servo5.setPosition(arm2);
            servo2.setPosition(arm3);
            servo3.setPosition(arm_dir);
            servo4.setPosition(arm_release);
            
            while (!touch.isPressed() || back_tick < 200) {
                back_tick += 10;
                r1motor.setTargetPosition(straight_tick - turn_degree_90_tick - back_tick);
                r1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                r1motor.setPower(0.5);
                r2motor.setTargetPosition(straight_tick - turn_degree_90_tick - back_tick);
                r2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                r2motor.setPower(0.5);
                l1motor.setTargetPosition(straight_tick + turn_degree_90_tick - back_tick);
                l1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                l1motor.setPower(0.5);
                l2motor.setTargetPosition(straight_tick + turn_degree_90_tick - back_tick);
                l2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                l2motor.setPower(0.5);
                while (r1motor.isBusy() || r2motor.isBusy() || l1motor.isBusy() || l2motor.isBusy()) {
                }
            }
            
            servo4.setPosition(arm_grab);
            t = getRuntime();
            while (getRuntime() - t < 2) {
            }
            
            arm1 = 280;
            motor1.setTargetPosition(arm1);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setPower(0.5);
            motor2.setTargetPosition(arm1);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setPower(0.5);
            
            t = getRuntime();
            while (getRuntime() - t < 1.5) {
            }
            
            r1motor.setTargetPosition(straight_tick - turn_degree_90_tick);
            r1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r1motor.setPower(1);
            l1motor.setTargetPosition(straight_tick + turn_degree_90_tick);
            l1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l1motor.setPower(1);
            r2motor.setTargetPosition(straight_tick - turn_degree_90_tick);
            r2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r2motor.setPower(1);
            l2motor.setTargetPosition(straight_tick + turn_degree_90_tick);
            l2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l2motor.setPower(1);
            while (r1motor.isBusy() || r2motor.isBusy() || l1motor.isBusy() || l2motor.isBusy()) {
            }
            
            arm1 = 819;
            arm2 = 0.56;
            arm3 = 0.37;
            arm_dir = 0.69;
            
            motor1.setTargetPosition(arm1);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setPower(0.2);
            motor2.setTargetPosition(arm1);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setPower(0.2);
            
            t = getRuntime();
            while (getRuntime() - t < 1) {
            }
            
            servo1.setPosition(arm2);
            servo5.setPosition(arm2);
            servo2.setPosition(arm3);
            servo3.setPosition(arm_dir);
            
            r1motor.setTargetPosition(straight_tick - turn_degree_90_tick + turn_degree_45_tick);
            r1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r1motor.setPower(1);
            l1motor.setTargetPosition(straight_tick + turn_degree_90_tick - turn_degree_45_tick);
            l1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l1motor.setPower(1);
            r2motor.setTargetPosition(straight_tick - turn_degree_90_tick + turn_degree_45_tick);
            r2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r2motor.setPower(1);
            l2motor.setTargetPosition(straight_tick + turn_degree_90_tick - turn_degree_45_tick);
            l2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l2motor.setPower(1);
            
            while (r1motor.isBusy() || r2motor.isBusy() || l1motor.isBusy() || l2motor.isBusy()) {
            }
            
            //release
            
            t = getRuntime();
            while (getRuntime() - t < 0.5) {
            }
            
            servo4.setPosition(0.1);
            t = getRuntime();
            while (getRuntime() - t < 0.5) {
            }
            
            arm1 = 800;
            motor1.setTargetPosition(arm1);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setPower(0.5);
            motor2.setTargetPosition(arm1);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setPower(0.5);
            t = getRuntime();
            while (getRuntime() - t < 0.5) {
            }
            
            servo1.setPosition(0.9);
            servo5.setPosition(0.9);
            servo2.setPosition(0.5);
            servo3.setPosition(0.03);
            servo4.setPosition(0.1);
        
            motor1.setTargetPosition(50);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setPower(0.2);
            motor2.setTargetPosition(50);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setPower(0.2);
            
            r1motor.setTargetPosition(straight_tick - turn_degree_90_tick);
            r1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r1motor.setPower(1);
            l1motor.setTargetPosition(straight_tick + turn_degree_90_tick);
            l1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l1motor.setPower(1);
            r2motor.setTargetPosition(straight_tick - turn_degree_90_tick);
            r2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r2motor.setPower(1);
            l2motor.setTargetPosition(straight_tick + turn_degree_90_tick);
            l2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l2motor.setPower(1);
            while (r1motor.isBusy() || r2motor.isBusy() || l1motor.isBusy() || l2motor.isBusy()) {
            }
            */
            
            int parking_tick;
            
            if (isDetected == "1 Bolt") {
                //left
                parking_tick = 920;
                r1motor.setTargetPosition(straight_tick - turn_degree_90_tick + parking_tick);
                r1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                r1motor.setPower(1);
                l1motor.setTargetPosition(straight_tick + turn_degree_90_tick + parking_tick);
                l1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                l1motor.setPower(1);
                r2motor.setTargetPosition(straight_tick - turn_degree_90_tick + parking_tick);
                r2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                r2motor.setPower(1);
                l2motor.setTargetPosition(straight_tick + turn_degree_90_tick + parking_tick);
                l2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                l2motor.setPower(1);
                while (r1motor.isBusy() || r2motor.isBusy() || l1motor.isBusy() || l2motor.isBusy()) {
                }
                telemetry.addData("left", "");
            } else if (isDetected == "2 Bulb") {
                //middle
                telemetry.addData("middle", "");
            } else if (isDetected == "3 Panel") {
                //right
                parking_tick = -920;
                r1motor.setTargetPosition(straight_tick - turn_degree_90_tick + parking_tick);
                r1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                r1motor.setPower(1);
                l1motor.setTargetPosition(straight_tick + turn_degree_90_tick + parking_tick);
                l1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                l1motor.setPower(1);
                r2motor.setTargetPosition(straight_tick - turn_degree_90_tick + parking_tick);
                r2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                r2motor.setPower(1);
                l2motor.setTargetPosition(straight_tick + turn_degree_90_tick + parking_tick);
                l2motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                l2motor.setPower(1);
                while (r1motor.isBusy() || r2motor.isBusy() || l1motor.isBusy() || l2motor.isBusy()) {
                }
                telemetry.addData("right", "");
            }
            telemetry.update();
        }
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

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.70f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        //tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
