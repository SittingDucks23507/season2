package org.firstinspires.ftc.teamcode.Auto;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Basic Parking", group="Robot")
public class Basic extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor frontLeft, frontRight, backLeft, backRight;
        final float SPEED = 0.25f;

        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontLeft.setDirection(REVERSE);
        backLeft.setDirection(REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            frontLeft.setPower(SPEED);
            frontRight.setPower(SPEED);
            backLeft.setPower(SPEED);
            backRight.setPower(SPEED);
            sleep(50);
        }
    }
}
