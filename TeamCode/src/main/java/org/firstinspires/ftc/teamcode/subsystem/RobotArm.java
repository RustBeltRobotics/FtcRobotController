package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

    private DcMotorEx intake1 = null;

    public RobotArm(HardwareMap hardwareMap, ElapsedTime runtime, Gamepad gamepad2, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.runtime = runtime;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
    }

    public void init() {
        arm1 = hardwareMap.get(DcMotorEx.class, "A1");
        arm2 = hardwareMap.get(DcMotorEx.class, "A2");

        arm1.setDirection(DcMotorEx.Direction.FORWARD);
        arm2.setDirection(DcMotorEx.Direction.REVERSE);

        arm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //intake
        intake1 = hardwareMap.get(DcMotorEx.class, "I1");

        intake1.setDirection(DcMotorEx.Direction.FORWARD);

        intake1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Arm Initialized");
    }

    public void loop() {
        //var for arm power value
        double powerA;
        double powerI;

        //set arm power to the stick output
        powerA = Range.clip(gamepad2.left_stick_y, -1.0, 1.0);
        arm1.setPower(powerA);
        arm2.setPower(powerA);

        powerI = Range.clip(gamepad2.left_trigger, -1.0, 0) + Range.clip(gamepad2.right_trigger, 0, 1);
        intake1.setPower(powerI);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("ArmMotors", "power:", powerA);
    }
}
