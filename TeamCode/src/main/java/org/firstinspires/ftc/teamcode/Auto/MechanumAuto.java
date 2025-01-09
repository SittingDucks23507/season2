package org.firstinspires.ftc.teamcode.Auto;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MechanumAuto {
    public LinearOpMode lop;
    // Directions to be used with movetime()
    public final int[][] FORWARD = { {1, 1},
            {1, 1} };
    public final int[][] LEFT    = { {-1, 1},
            {1, -1} };

    public final int[][] DOWN    = dtoim(mulsm(FORWARD, -1));
    public final int[][] RIGHT   = dtoim(mulsm(LEFT, -1));

    // define motors
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    // define runtime, used in movetime()
    private ElapsedTime runtime = new ElapsedTime();

    // Motor SPEED, adjust as necessary
    private final float SPEED = 0.25f;
    private final float TPR = ((((1+(46/17))) * (1+(46/11))) * 28); // TODO: Update to actual value
    private final float RPI = 1.0f; // TODO: find out

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
    public void movetime(float time, int[][] direction) {
        double[][] movevec = mulsm(direction, SPEED);

        frontLeft.setPower(movevec[0][0]);
        frontRight.setPower(movevec[0][1]);
        backLeft.setPower(movevec[1][0]);
        backRight.setPower(movevec[1][1]);

        runtime.reset();
        while (lop.opModeIsActive() && (runtime.seconds() < time)) {
            lop.telemetry.addData("time", runtime.seconds());
            lop.telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    public void moveticks(float inches, int[][] direction) {
        frontLeft.setTargetPosition((int)(inches * TPR * RPI) * direction[0][0]);
        frontRight.setTargetPosition((int)(inches * TPR * RPI) * direction[0][1]);
        backLeft.setTargetPosition((int)(inches * TPR * RPI) * direction[1][0]);
        backRight.setTargetPosition((int)(inches * TPR * RPI) * direction[1][1]);

        frontLeft.setMode(RUN_TO_POSITION);
        frontRight.setMode(RUN_TO_POSITION);
        backLeft.setMode(RUN_TO_POSITION);
        backRight.setMode(RUN_TO_POSITION);
        frontLeft.setPower(SPEED);
        frontRight.setPower(SPEED);
        backLeft.setPower(SPEED);
        backRight.setPower(SPEED);
        while (lop.opModeIsActive() &&
                frontLeft.isBusy() &&
                frontRight.isBusy() &&
                backLeft.isBusy() &&
                backRight.isBusy()) {
            lop.telemetry.addData("Target (FLFRBLBR)", "%7d, %7d, %7d, %7d",
                    frontLeft.getTargetPosition(),
                    frontRight.getTargetPosition(),
                    backLeft.getTargetPosition(),
                    backRight.getTargetPosition());

            lop.telemetry.addData("Position (FLFRBLBR)", "%7d, %7d, %7d, %7d",
                    frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition(),
                    backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition());
            lop.telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
