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

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    @Override
    public void runOpMode() {
        DcMotor  frontLeft, frontRight, backLeft, backRight;
        DcMotor leftSlide, rightSlide, arm, b;

        Servo wrist;
        CRServo finger;

        // Define Motors
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        arm = hardwareMap.get(DcMotor.class, "las");
        b = hardwareMap.get(DcMotor.class, "la");

        wrist = hardwareMap.get(Servo.class, "wrist");
        finger = hardwareMap.get(CRServo.class, "finger");

        frontLeft.setDirection(REVERSE);
        backLeft.setDirection(REVERSE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        wrist.setDirection(Servo.Direction.REVERSE);
        waitForStart();

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double lsY, lsX, rsX;
            float speed = 1f;
//            float armm = 0f;
            float bm = 0f;
            final float SLIDE_POWER = 1f;
            finger.setPower(0);

            if (gamepad1.right_bumper) {
                speed = 0.25f;
            }
            bm -= gamepad2.right_stick_y;

            lsY = -gamepad1.left_stick_y; // W_UP is now positive
            lsX = gamepad1.left_stick_x;
            rsX = gamepad1.right_stick_x; // un-backwards it
            frontLeft.setPower((lsY + lsX + rsX)  * speed);
            frontRight.setPower((lsY - lsX - rsX) * speed);
            backLeft.setPower((lsY - lsX + rsX)   * speed);
            backRight.setPower((lsY + lsX - rsX)  * speed);

            if (gamepad2.dpad_up)
                wrist.setPosition(0);
            if (gamepad2.dpad_down)
                wrist.setPosition(.4);

            if (gamepad2.cross)
                finger.setPower(1);
            if (gamepad2.square)
                finger.setPower(-1);

            arm.setTargetPosition(arm.getCurrentPosition() + (int)(100 * -gamepad2.left_stick_y));
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.5);

            b.setPower(bm * 0.5);

            telemetry.addData("Front Left", frontLeft);
            telemetry.addData("Front Right", frontRight);
            telemetry.addData("Back Left", backLeft);
            telemetry.addData("Back Right", backRight);
            telemetry.addData("Arm", arm.getCurrentPosition());
            telemetry.addData("b", b.getPower());
            telemetry.addLine("----------");
            telemetry.addData("Wrist", wrist.getPosition());
            telemetry.addData("Finger", finger.getPower());
            telemetry.update();
            // reasonable speed
            sleep(50);
        }
    }
}
