package org.firstinspires.ftc.teamcode.Auto;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="MoveTicks (Test)", group="zTests")
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        MechanumAuto Auto = new MechanumAuto();

        Auto.frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        Auto.frontRight = hardwareMap.get(DcMotor.class, "front_right");
        Auto.backLeft = hardwareMap.get(DcMotor.class, "back_left");
        Auto.backRight = hardwareMap.get(DcMotor.class, "back_right");

        Auto.frontLeft.setDirection(REVERSE);
        Auto.backLeft.setDirection(REVERSE);
        Auto.lop = this;
        waitForStart();
        Auto.moveticks(12, Auto.FORWARD);
    }
}
