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
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRW_UPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="mechanum drive", group="Robot")
public class MechanumDrive extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  frontLeft, frontRight, backLeft, backRight, armMotor, leftSlide, rightSlide;
    public Servo wrist;
    public CRServo finger;

    @Override
    public void runOpMode() {
        // Wrist
        final double W_UP = 0.55;
        final double W_CENTER = 0.25;
        final double W_DOWN = 0;
        // Arm
        final int A_UP = -550;
        final int A_DOWN = -50;
        final int A_STORE = -300;
        
        final float armPower = 0.1f;

        // Define Motors
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        leftSlide = hardwareMap.get(DcMotor.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotor.class, "right_slide");

        armMotor = hardwareMap.get(DcMotor.class, "intake_arm");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Define Servos
        wrist = hardwareMap.get(Servo.class, "wrist");
        finger = hardwareMap.get(CRServo.class, "finger");

        frontLeft.setDirection(REVERSE);
        backLeft.setDirection(REVERSE);
        waitForStart();
        wrist.setPosition(W_CENTER);
        armMotor.setTargetPosition(A_STORE);
        armMotor.setMode(RUN_TO_POSITION);
        armMotor.setPower(armPower);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double lsY, lsX, rsX;
            finger.setPower(0);

            lsY = -gamepad1.left_stick_y; // W_UP is now positive
            lsX = gamepad1.left_stick_x;
            rsX = gamepad1.right_stick_x; // un-backwards it
            frontLeft.setPower(lsY + lsX + rsX);
            frontRight.setPower(lsY - lsX - rsX);
            backLeft.setPower(lsY - lsX + rsX);
            backRight.setPower(lsY + lsX - rsX);

            /*
             * Operator (gamepad2)
             */
            // Arm
            if (gamepad2.right_bumper) {
                armMotor.setTargetPosition(A_DOWN);
                armMotor.setMode(RUN_TO_POSITION);
                armMotor.setPower(armPower);
            }
            if (gamepad2.left_bumper) {
                armMotor.setTargetPosition(A_UP);
                armMotor.setMode(RUN_TO_POSITION);
                armMotor.setPower(armPower);
            }
            if (gamepad2.ps) {
                armMotor.setTargetPosition(A_STORE);
                armMotor.setMode(RUN_TO_POSITION);
                armMotor.setPower(armPower);
            }

            // Wrist
            if (gamepad2.dpad_up) {
                wrist.setPosition(W_UP);
            }
            if (gamepad2.dpad_down) {
                wrist.setPosition(W_DOWN);
            }
            if (gamepad2.dpad_left || gamepad2.dpad_right) {
                wrist.setPosition(W_CENTER);
            }

            if (gamepad2.circle){
                finger.setPower(1);
            }

            if (gamepad2.cross) {
                finger.setPower(-1);
            }
            if (gamepad2.triangle) {
                    leftSlide.setTargetPosition(-4300);
                    leftSlide.setMode(RUN_TO_POSITION);
                    leftSlide.setPower(0.2);

                    rightSlide.setTargetPosition(4300);
                    rightSlide.setMode(RUN_TO_POSITION);
                    rightSlide.setPower(0.2);
            } if (gamepad2.square) {
                leftSlide.setTargetPosition(-50);
                leftSlide.setMode(RUN_TO_POSITION);
                leftSlide.setPower(0.2);

                rightSlide.setTargetPosition(50);
                rightSlide.setMode(RUN_TO_POSITION);
                rightSlide.setPower(0.2);
            }

//            telemetry.addData("Arm Motor", armMotor.getCurrentPosition());
            telemetry.addData("Hand Servo", wrist.getPosition());
            telemetry.addData("Finger Servo (Continuous)", finger.getPower());
            telemetry.addData("arm", armMotor.getCurrentPosition());
            telemetry.update();

            // reasonable speed
            sleep(50);
        }
    }
}