package org.firstinspires.ftc.teamcode.Auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Basket (Test)", group="zTests")
public class Basket extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        MechanumAuto Auto = new MechanumAuto();
        DcMotor leftSlide, rightSlide, arm, b;
        Servo wrist;
        CRServo finger;

        int[][] TURN_LEFT =  { {-1, 1}, {-1, 1} };

        Auto.frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        Auto.frontRight = hardwareMap.get(DcMotor.class, "front_right");
        Auto.backLeft = hardwareMap.get(DcMotor.class, "back_left");
        Auto.backRight = hardwareMap.get(DcMotor.class, "back_right");

        leftSlide = hardwareMap.get(DcMotor.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotor.class, "right_slide");

        arm = hardwareMap.get(DcMotor.class, "las");
        b = hardwareMap.get(DcMotor.class, "la");

        wrist = hardwareMap.get(Servo.class, "wrist");
        finger = hardwareMap.get(CRServo.class, "finger");

        arm.setZeroPowerBehavior(BRAKE);

        Auto.frontLeft.setDirection(REVERSE);
        Auto.backLeft.setDirection(REVERSE);
        Auto.lop = this;

        waitForStart();
        wrist.setPosition(0);

        arm.setTargetPosition(arm.getCurrentPosition() + 500);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
        runtime.reset();
        while ((opModeIsActive() && arm.isBusy()) || (runtime.seconds() < 3)) {
            telemetry.addData("arm", arm.getCurrentPosition());
        }
        arm.setTargetPosition(arm.getCurrentPosition() - 600);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);

        // Slides go four seconds upwards.
        // Arm goes six seconds upwards, 5½ are taken by the move instructions.
        rightSlide.setPower(1f);
        leftSlide.setPower(-1f);
        b.setPower(0.5);

        Auto.movetime(2, Auto.FORWARD);
        Auto.movetime(2, Auto.RIGHT);

        rightSlide.setPower(0);
        leftSlide.setPower(0);

        Auto.movetime(1, TURN_LEFT);
        Auto.movetime(0.5f, Auto.FORWARD);
        // sleep is worse than waiting in a while() loop, good enough though as this shoudln't take
        // over time
        sleep(500);
        b.setPower(0);

        arm.setTargetPosition(arm.getCurrentPosition() - 200);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
        runtime.reset();
        while ((opModeIsActive() && arm.isBusy()) || (runtime.seconds() < 5)) {
            telemetry.addData("arm", arm.getCurrentPosition());
        }
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm.setPower(0);

        finger.setPower(1);
        sleep(500);
    }
}
