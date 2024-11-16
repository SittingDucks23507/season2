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

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

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

@TeleOp(name="slide test", group="Tests")
public class SlideTest extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  leftSlide, rightSlide;
    final double SPEED = 0.25;

    @Override
    public void runOpMode() {
        leftSlide = hardwareMap.get(DcMotor.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotor.class, "right_slide");
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                leftSlide.setTargetPosition(leftSlide.getCurrentPosition() + 50);
                leftSlide.setMode(RUN_TO_POSITION);
                leftSlide.setPower(0.2);
            } if (gamepad1.dpad_down) {
                leftSlide.setTargetPosition(leftSlide.getCurrentPosition() - 50);
                leftSlide.setMode(RUN_TO_POSITION);
                leftSlide.setPower(0.2);
            }
            if (gamepad1.triangle) {
                rightSlide.setTargetPosition(rightSlide.getCurrentPosition() - 50);
                rightSlide.setMode(RUN_TO_POSITION);
                rightSlide.setPower(0.2);
            } if (gamepad1.cross) {
                rightSlide.setTargetPosition(rightSlide.getCurrentPosition() + 50);
                rightSlide.setMode(RUN_TO_POSITION);
                rightSlide.setPower(0.2);
            }

            if (gamepad1.square) {
                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (gamepad1.circle) {
                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            telemetry.addData("Left Slide", leftSlide.getCurrentPosition());
            telemetry.addData("Right Slide", rightSlide.getCurrentPosition());
            telemetry.update();

            // reasonable speed
            sleep(50);
        }
    }
}