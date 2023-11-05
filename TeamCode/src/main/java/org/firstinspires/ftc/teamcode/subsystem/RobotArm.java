package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
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
    private DcMotor arm1 = null;
    private DcMotor arm2 = null;

    public RobotArm(HardwareMap hardwareMap, ElapsedTime runtime, Gamepad gamepad2, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.runtime = runtime;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
    }

    public void init() {
        arm1 = hardwareMap.get(DcMotor.class, "A1");
        arm2 = hardwareMap.get(DcMotor.class, "A2");

        arm1.setDirection(DcMotor.Direction.FORWARD);
        arm2.setDirection(DcMotor.Direction.REVERSE);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Arm Initialized");
    }

    public void loop() {
        //var for arm power value
        double powerA;

        //set arm power to the stick output
        powerA = Range.clip(gamepad2.left_stick_y, -1.0, 1.0);
        System.out.print(powerA);
        arm1.setPower(powerA);
        arm2.setPower(powerA);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("ArmMotors", "power:", powerA);
    }
}
