package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class RobotArm {
    private ElapsedTime runtime;

    private HardwareMap hardwareMap;

    private DcMotor arm1 = null;
    private DcMotor arm2 = null;

    public RobotArm(HardwareMap hardwareMap, ElapsedTime runtime) {
        this.hardwareMap = hardwareMap;
        this.runtime = runtime;
    }

    public void init() {
        arm1 = hardwareMap.get(DcMotor.class, "A1");
        arm2 = hardwareMap.get(DcMotor.class, "A2");

        arm1.setDirection(DcMotor.Direction.FORWARD);
        arm2.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Arm Initialized");
    }

    public void loop() {
        //var for arm power value
        double powerA;

        //set arm power to the stick output
        powerA = Range.clip(gamepad2.left_stick_y, -1.0, 1.0);
        arm1.setPower(powerA);
        arm2.setPower(powerA);

        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("ArmMotors", "power:", powerA);
    }
}
