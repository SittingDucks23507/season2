package org.firstinspires.ftc.teamcode.Auto;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// TODO: movetime() on its own, to be included in other classes.

@Autonomous(name="Drifting Auto", group="Experimental")
public class MechanumAuto extends LinearOpMode {
    // Directions to be used with movetime()
    private final int[][] FORWARD = { {1, 1},
                                      {1, 1} };
    private final int[][] LEFT    = { {-1, 1},
                                      {1, -1} };

    private final int[][] DOWN    = dtoim(mulsm(FORWARD, -1));
    private final int[][] RIGHT   = dtoim(mulsm(LEFT, -1));

    // define motors
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    // define runtime, used in movetime()
    private ElapsedTime runtime = new ElapsedTime();

    // Motor SPEED, adjust as necessary
    private final float SPEED = 0.25f;

    // Takes an integer matrix ‘m’ and multiplies it by scalar ‘c’
    private double[][] mulsm(int[][] m, double c) {
        double[][] r = new double[2][2];
        r[0][0] = m[0][0] * c;
        r[0][1] = m[0][1] * c;
        r[1][0] = m[1][0] * c;
        r[1][1] = m[1][1] * c;

        return r;
    }
    // Converts double[][] to int[][]
    // Should the directions just be doubles? Probably.
    private int[][] dtoim(double[][] m) {
        int[][] r = new int[2][2];
        r[0][0] = (int)m[0][0];
        r[0][1] = (int)m[0][1];
        r[1][0] = (int)m[1][0];
        r[1][1] = (int)m[1][1];

        return r;
    }

    // Runs motors at SPEED power for ‘time’ seconds in ‘direction’
    // Directions as defined above ‘FORWARD’ ‘DOWN’ ‘LEFT’ ‘RIGHT’
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
        // Initalize motors
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontLeft.setDirection(REVERSE);
        backLeft.setDirection(REVERSE);

        waitForStart();

        // Sample instructions, drives forward for three seconds and left for two
        movetime(3.0f, FORWARD);
        movetime(2.0f, LEFT);
        sleep(50);
    }
}
