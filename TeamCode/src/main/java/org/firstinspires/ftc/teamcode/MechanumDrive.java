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

package org.firstinspires.ftc.teamcode;

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
    public DcMotor  frontLeft, frontRight, backLeft, backRight, armMotor;
    public Servo hand;
    public CRServo finger;

    @Override
    public void runOpMode() {
        final float handRight = 0.2f;
        final float handCenter = 0.55f;
        final float handLeft = 0.9f;

        // Define Motors
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Define Servos
        hand = hardwareMap.get(Servo.class, "hand");
        finger = hardwareMap.get(CRServo.class, "finger");

        hand.setPosition(handCenter);
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double lsY, lsX, rsX;
            finger.setPower(0);

            lsY = gamepad1.left_stick_y; // Up is now positive
            lsX = gamepad1.left_stick_x;
            rsX = -gamepad1.right_stick_x; // un-backwards it
            frontLeft.setPower(lsY + lsX + rsX);
            frontRight.setPower(lsY - lsX - rsX);
            backLeft.setPower(lsY - lsX + rsX);
            backRight.setPower(lsY + lsX - rsX);

            if (gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1) {
                int encpos = (int)-gamepad2.left_stick_y * 100 + armMotor.getCurrentPosition();
                armMotor.setTargetPosition(encpos); // TODO: change this to the actual pos
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.2);
            }

            if (gamepad2.dpad_right) {
                hand.setPosition(handRight);
            }
            if (gamepad2.dpad_up) {
                hand.setPosition(handCenter);
            }
            if (gamepad2.dpad_left){
                hand.setPosition(handLeft);
            }

            if (gamepad2.a){
                finger.setPower(1);
            }

            if (gamepad2.b) {
                finger.setPower(-1);
            }

            telemetry.addData("Arm Motor", armMotor.getCurrentPosition());
            telemetry.addData("Hand Servo", hand.getPosition());
            telemetry.addData("Finger Servo (Continuous)", finger.getPower());
            telemetry.update();

            // reasonable speed
            sleep(50);
        }
    }
}