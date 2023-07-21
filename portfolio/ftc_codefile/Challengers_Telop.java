/*
Copyright 2023 FIRST Tech Challenge Team FTC

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class Challengers_Telop extends LinearOpMode {
    private Blinker control_Hub;
    private Blinker expansion_Hub_3;
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

    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_3 = hardwareMap.get(Blinker.class, "Expansion Hub 3");
        imu = hardwareMap.get(IMU.class, "imu");
        r1motor = hardwareMap.get(DcMotor.class, "r1motor");
        r2motor = hardwareMap.get(DcMotor.class, "r2motor");
        l1motor = hardwareMap.get(DcMotor.class, "l1motor");
        l2motor = hardwareMap.get(DcMotor.class, "l2motor");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        r1motor.setDirection(DcMotor.Direction.REVERSE);
        r2motor.setDirection(DcMotor.Direction.REVERSE);
        l1motor.setDirection(DcMotor.Direction.FORWARD);
        l2motor.setDirection(DcMotor.Direction.FORWARD);
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        
        r1motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r2motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l1motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l2motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");
        servo5 = hardwareMap.get(Servo.class, "servo5");
        
        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.FORWARD);
        servo3.setDirection(Servo.Direction.FORWARD);
        servo4.setDirection(Servo.Direction.FORWARD);
        servo5.setDirection(Servo.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        double m_p1 = 0.2;
        double sv_p2 = 0.9;
        double sv_p3 = 0.5;
        double sv_p4 = 0.03;
        int sv_s4 = 0;
        double sv_p5 = 0.1;
        int sv_s5 = 0;
        int motor_tick = 0;
        double t1 = 0;
        double t2 = 0;

        while (opModeIsActive()) {
            double x1_1 = gamepad1.right_stick_x;
            double y1_1 = -gamepad1.left_stick_y;
            double x1_2 = gamepad1.left_stick_x;
            r1motor.setPower(0.5*(y1_1-x1_1-x1_2));
            r2motor.setPower(0.5*(y1_1+x1_1-x1_2));
            l1motor.setPower(0.5*(y1_1+x1_1+x1_2));
            l2motor.setPower(0.5*(y1_1-x1_1+x1_2));

            double y1 = gamepad2.left_stick_y;
            double y2 = gamepad2.right_stick_y;
            if (y1 > 0) {
                motor_tick -= 3;
                m_p1 = -0.3;
            } else if (y1 < 0) {
                motor_tick += 3;
                m_p1 = 0.3;
            }
            if (y2 < 0) {
                sv_p2 += 0.004;
                if (sv_p2 > 1) {
                    sv_p2 = 1;
                }
            } else if (y2 > 0){
                sv_p2 -= 0.004;
                if (sv_p2 < 0) {
                    sv_p2 = 0;
                }
            }
            if (gamepad2.dpad_up) {
                sv_p3 += 0.007;
                if (sv_p3 > 1) {
                    sv_p3 = 1;
                }
            } else if (gamepad2.dpad_down) {
                sv_p3 -= 0.007;
                if (sv_p3 < 0) {
                    sv_p3 = 0;
                }
            }
            if (gamepad2.right_bumper && getRuntime() - t1 > 1) {
                if (sv_s4 == 0) {
                    sv_p4 = 0.69;
                    sv_s4 = 1;
                } else {
                    sv_p4 = 0.03;
                    sv_s4 = 0;
                }
                t1 = getRuntime();
            }
            if (gamepad2.left_bumper&& getRuntime() - t2 > 0.5) {
                if (sv_s5 == 0) {
                    sv_p5 = 0.1;
                    sv_s5 = 1;
                } else {
                    sv_p5 = 0.5;
                    sv_s5 = 0;
                }
                t2 = getRuntime();
            }
            if (gamepad2.y) {
                motor_tick = 624;
                sv_p2 = 0.558;
                sv_p3 = 0.3459;
                sv_p4 = 0.69;
            } else if (gamepad2.x) {
                motor_tick = 657;
                sv_p2 = 0.88;
                sv_p3 = 0.68;
                sv_p4 = 0.69;
            } else if (gamepad2.a) {
                motor_tick = 885;
                sv_p2 = 1;
                sv_p3 = 0.71;
                sv_p4 = 0.69;
            } else if (gamepad2.b) {
                motor_tick = 81;
                sv_p2 = 0.5379;
                sv_p3 = 0.3179;
                sv_p4 = 0.03;
            }

            motor1.setTargetPosition(motor_tick);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setPower(m_p1);
            motor2.setTargetPosition(motor_tick);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setPower(m_p1);
            servo1.setPosition(sv_p2);
            servo5.setPosition(sv_p2);
            servo2.setPosition(sv_p3);
            servo3.setPosition(sv_p4);
            servo4.setPosition(sv_p5);
            
            telemetry.update();
        }
    }
}
