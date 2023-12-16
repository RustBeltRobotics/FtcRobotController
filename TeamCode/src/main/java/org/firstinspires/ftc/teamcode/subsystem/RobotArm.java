package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Subsystem for Arm mechanism (extension, rotation, etc.)
 */
public class RobotArm {
    private final ElapsedTime runtime;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad2;
    private DcMotorEx arm1 = null;
    private DcMotorEx arm2 = null;
    private DcMotorEx ext1 = null;

    private DcMotorEx intake1 = null;

    public static double ARM_AUTHORITY = .7;

    public RobotArm(HardwareMap hardwareMap, ElapsedTime runtime, Gamepad gamepad2, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.runtime = runtime;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
    }

    public void init() {
        arm1 = hardwareMap.get(DcMotorEx.class, "A1");
        arm2 = hardwareMap.get(DcMotorEx.class, "A2");
        intake1 = hardwareMap.get(DcMotorEx.class, "I1");
        ext1 = hardwareMap.get(DcMotorEx.class, "E1");

        arm1.setDirection(DcMotorEx.Direction.FORWARD);
        arm2.setDirection(DcMotorEx.Direction.REVERSE);
        ext1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake1.setDirection(DcMotorEx.Direction.FORWARD);

        arm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        ext1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intake1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Arm Initialized");
    }

    public void loop() {

        if (gamepad2.a == true) {
            moveArm("scoring");
        }


        RobotLog.dd("gamepad lx", String.valueOf(gamepad2.left_stick_y));
        //var for arm power value
        double powerA;
        double powerI;
        double powerE;

        //set arm power to the stick output
        powerA = (Range.clip(gamepad2.left_stick_y, -1.0, 1.0) * ARM_AUTHORITY);
        arm1.setPower(powerA);
        arm2.setPower(-powerA);

        powerE = Range.clip(gamepad2.right_stick_y, -1.0, 1.0);
        ext1.setPower(-powerE);

        powerI = Range.clip(-gamepad2.left_trigger, -1.0, 0) + Range.clip(gamepad2.right_trigger, 0, 1);
        intake1.setPower(powerI);
        if (gamepad2.left_trigger > .01) {
            arm1.setPower(-.2);
            arm2.setPower(-.2);
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("ArmMotors", "power:", powerA);
    }

    private void moveArm(String thingToDo) {
        int scoringPos = -324;
        int intakePos = 35;

        if (thingToDo.equals("scoring")) {
            arm1.setTargetPosition(scoringPos);
            arm1.setPower(.5);
            while (arm1.isBusy()) {
                arm2.setPower(.5);
                arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            arm2.setPower(0);
        } else if (thingToDo.equals("intake")) {
            arm1.setTargetPosition(intakePos);
            arm1.setPower(.5);
            while (arm1.isBusy()) {
                arm2.setPower(.5);
                arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            arm2.setPower(0);
        } else if (thingToDo.equals("offGround")) {
            arm1.setTargetPosition(300);
            arm1.setPower(.5);
            while (arm1.isBusy()) {
                arm2.setPower(.5);
                arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            arm2.setPower(0);
        } else {
            telemetry.addData("moveArm() err:", "invalid value given");
            telemetry.update();
        }
    }
}
