package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class RobotDrive {

    private HardwareMap hardwareMap;
    private ElapsedTime runtime;
    private DcMotor left1 = null;
    private DcMotor left2 = null;
    //define right motors 1 and 2
    private DcMotor right1 = null;
    private DcMotor right2 = null;

    public RobotDrive(HardwareMap hardwareMap, ElapsedTime runtime ) {
        this.hardwareMap = hardwareMap;
        this.runtime = runtime;
    }

    public void init() {
        left1 = hardwareMap.get(DcMotor.class, "L1");
        left2 = hardwareMap.get(DcMotor.class, "L2");

        right1 = hardwareMap.get(DcMotor.class, "R1");
        right2 = hardwareMap.get(DcMotor.class, "R2");
        //set default motor directions
        left1.setDirection(DcMotor.Direction.FORWARD);
        left2.setDirection(DcMotor.Direction.REVERSE);

        right1.setDirection(DcMotor.Direction.FORWARD);
        right2.setDirection(DcMotor.Direction.REVERSE);
    }
    public void loop() {
        double powerL;
        double powerR;

        //fetch the stick data and do the math
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        powerL = Range.clip(drive + turn, -1.0, 1.0);
        powerR = Range.clip(drive - turn, -1.0, 1.0);

        //send the resultant to the motors
        left1.setPower(powerL);
        left2.setPower(powerL);
        right1.setPower(powerR);
        right2.setPower(powerR);

        //log that!
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("DriveMotors", "left (%.2f), right (%.2f)", powerL, powerR);
    }
}
