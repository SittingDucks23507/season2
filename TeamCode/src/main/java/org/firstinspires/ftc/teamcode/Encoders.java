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

@TeleOp(name="Arm Calibrate", group="Calibrations")
public class Encoders extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor armMotor;
    public Servo hand;

    @Override
    public void runOpMode() {

        // Define and Initialize Motors
        armMotor = hardwareMap.get(DcMotor.class, "intake_arm");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            armMotor.setPower(bti(gamepad1.dpad_up) - bti(gamepad1.dpad_down));
            if (gamepad1.dpad_up) {
                armMotor.setTargetPosition(armMotor.getCurrentPosition() + 100);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.2);
            }
            if (gamepad1.dpad_down) {
                armMotor.setTargetPosition(armMotor.getCurrentPosition() - 100);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.2);
            }
            if (gamepad1.cross) {
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            telemetry.addData("Arm Motor", armMotor.getCurrentPosition());
            telemetry.addData("lsy", gamepad2.left_stick_y);
            telemetry.update();

            // reasonable speed
            sleep(50);
        }
    }
}
