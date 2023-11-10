package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.model.DriveType;

/**
 * Subsystem for Drivetrain
 */
public class RobotDrive {

    private final HardwareMap hardwareMap;
    private final ElapsedTime runtime;
    private final Telemetry telemetry;
    private final Gamepad gamepad1;
    private DcMotor left1 = null;
    private DcMotor left2 = null;
    //define right motors 1 and 2
    private DcMotor right1 = null;
    private DcMotor right2 = null;

    private DriveType driveType = DriveType.TANK;

    public RobotDrive(HardwareMap hardwareMap, ElapsedTime runtime, Gamepad gamepad1, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.runtime = runtime;
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
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

        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void drive() {
        double powerL;
        double powerR;

        if (driveType == DriveType.ARCADE) {
            double drive = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            powerL = Range.clip(drive + turn, -1.0, 1.0);
            powerR = Range.clip(drive - turn, -1.0, 1.0);
        } else if (driveType == DriveType.TANK) {
            powerL = -gamepad1.left_stick_y;
            powerR = -gamepad1.right_stick_y;
        } else {
            throw new IllegalArgumentException("Unexpected drive type: " + driveType);
        }

        //send the resultant to the motors
        left1.setPower(powerL);
        left2.setPower(powerL);
        right1.setPower(powerR);
        right2.setPower(powerR);

        //log that!
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("DriveMotors", "left (%.2f), right (%.2f)", powerL, powerR);
    }

    public DriveType getDriveType() {
        return driveType;
    }

    public void setDriveType(DriveType driveType) {
        this.driveType = driveType;
    }
}