package org.firstinspires.ftc.teamcode.op.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.model.DriveType;
import org.firstinspires.ftc.teamcode.subsystem.April;
import org.firstinspires.ftc.teamcode.subsystem.RobotArm;
import org.firstinspires.ftc.teamcode.subsystem.RobotDrive;

public abstract class TeleOpParent extends OpMode {
    protected double autoBrakeDistance = 5.0; //placeholder value until field tuning is done :)

    protected final ElapsedTime runtime = new ElapsedTime();
    protected RobotArm robotArm = null;
    protected RobotDrive robotDrive = null;
    protected April april = null;
    protected boolean autoBrake = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        robotArm = new RobotArm(hardwareMap, runtime, gamepad2, telemetry);
        robotDrive = new RobotDrive(hardwareMap, runtime, gamepad1, telemetry);
        april = new April(hardwareMap, telemetry, gamepad1);

        telemetry.addData("Power Left val:", gamepad1.toString());
        telemetry.addData("Status", "Initialized");

        robotDrive.init();
        robotDrive.setDriveType(getDriveType());
        robotArm.init();
    }

    @Override
    public void loop() {
        autoBrake = april.autoBrake(autoBrakeDistance);
        robotArm.loop();
        robotDrive.drive(autoBrake);
    }

    public abstract DriveType getDriveType();
}
