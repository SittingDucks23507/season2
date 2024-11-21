package org.firstinspires.ftc.teamcode.Auto;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Drifting Auto", group="Experimental")
public class MechanumAuto extends LinearOpMode {
    private final int[][] FORWARD = { {1, 1},
            {1, 1} };
    private final int[][] LEFT    = { {-1, 1},
            {1, -1} };
    private final int[][] DOWN    = { {-1, -1},
            {-1, -1} };
    private final int[][] RIGHT   = { {1, -1},
            {-1, 1} };

    private double[][] mulsm(int[][] m, double c) {
        double[][] r = new double[2][2];
        r[0][0] = m[0][0] * c;
        r[0][1] = m[0][1] * c;
        r[1][0] = m[1][0] * c;
        r[1][1] = m[1][1] * c;

        return r;
    }

    public DcMotor frontLeft, frontRight, backLeft, backRight;
    private ElapsedTime runtime = new ElapsedTime();
    private final float SPEED = 0.1f;
    private void movetime(float time, int[][] direction) {
        double[][] movevec = mulsm(direction, SPEED);

        frontLeft.setPower(movevec[0][0]);
        frontRight.setPower(movevec[0][1]);
        backLeft.setPower(movevec[1][0]);
        backRight.setPower((movevec[1][1]));
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("time", runtime.seconds());
            telemetry.update();
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontLeft.setDirection(REVERSE);
        backLeft.setDirection(REVERSE);
        waitForStart();

        movetime(3.0f, FORWARD);
        movetime(2.0f, LEFT);
        sleep(50);
    }
}