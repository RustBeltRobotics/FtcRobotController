package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.model.DriveType;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Subsystem for Drivetrain
 */
public class RobotDrive {

    private final HardwareMap hardwareMap;
    private final ElapsedTime runtime;
    private final Telemetry telemetry;
    private final Gamepad gamepad1;
    private DcMotorEx left1 = null;
    private DcMotorEx left2 = null;
    //define right motors 1 and 2
    private DcMotorEx right1 = null;
    private DcMotorEx right2 = null;

    private DriveType driveType = DriveType.TANK;

    public RobotDrive(HardwareMap hardwareMap, ElapsedTime runtime, Gamepad gamepad1, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.runtime = runtime;
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    public void init() {
        left1 = hardwareMap.get(DcMotorEx.class, "L1");
        left2 = hardwareMap.get(DcMotorEx.class, "L2");

        right1 = hardwareMap.get(DcMotorEx.class, "R1");
        right2 = hardwareMap.get(DcMotorEx.class, "R2");
        //set default motor directions
        left1.setDirection(DcMotorEx.Direction.FORWARD);
        left2.setDirection(DcMotorEx.Direction.REVERSE);

        right1.setDirection(DcMotorEx.Direction.FORWARD);
        right2.setDirection(DcMotorEx.Direction.REVERSE);

        left1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        left1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        left1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }

    public void drive(boolean autoBrake) {
        double velL;
        double velR;
        if (autoBrake == false){
            if (driveType == DriveType.ARCADE) {
                double drive = gamepad1.left_stick_y;
                double turn = -gamepad1.right_stick_x;
                velL = Range.clip(drive + turn, -1.0, 1.0);
                velR = Range.clip(drive - turn, -1.0, 1.0);
            } else if (driveType == DriveType.TANK) {
                velL = -gamepad1.left_stick_y;
                velR = -gamepad1.right_stick_y;
            } else {
                throw new IllegalArgumentException("Unexpected drive type: " + driveType);
            }
        } else{
            if (driveType == DriveType.ARCADE) {
                double drive = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;
                velL = Range.scale(drive + turn, -1.0, 1.0, .2, -1);
                velR = Range.scale(drive - turn, -1.0, 1.0, .2, -1);
            } else if (driveType == DriveType.TANK) {
                velL = -gamepad1.left_stick_y;
                velR = -gamepad1.right_stick_y;
            } else {
                throw new IllegalArgumentException("Unexpected drive type: " + driveType);
            }
        }

        //send the resultant to the motors
        left1.setPower(velL);
        left2.setPower(velL);
        right1.setPower(velR);
        right2.setPower(velR);

        telemetry.addData("l1",left1.getVelocity());
        telemetry.addData("l2",left2.getVelocity());
        telemetry.addData("r1",right1.getVelocity());
        telemetry.addData("r2:",right2.getVelocity());

        //log that!
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("DriveMotors", "left (%.2f), right (%.2f)", velL, velR);
    }


    public DriveType getDriveType() {
        return driveType;
    }

    public void setDriveType(DriveType driveType) {
        this.driveType = driveType;
    }
}
