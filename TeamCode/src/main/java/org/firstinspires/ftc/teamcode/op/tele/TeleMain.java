package org.firstinspires.ftc.teamcode.op.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.model.DriveType;
import org.firstinspires.ftc.teamcode.subsystem.April;
import org.firstinspires.ftc.teamcode.subsystem.RobotArm;
import org.firstinspires.ftc.teamcode.subsystem.RobotDrive;

@TeleOp(name="Tele-Comp", group="Iterative OpMode")
//@Disabled
public class TeleMain extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private RobotArm robotArm = null;
    private RobotDrive robotDrive = null;
    public April april = null;
    private boolean autoBrake = false;
    private double brakeDistance = 5.0;
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        robotArm = new RobotArm(hardwareMap, runtime, gamepad2, telemetry);
        robotDrive = new RobotDrive(hardwareMap, runtime, gamepad1, telemetry);
        april = new April(hardwareMap, telemetry, gamepad1);

        telemetry.addData("Power Left val:", gamepad1.toString());
        telemetry.addData("Status", "Initialized");

        robotDrive.init();
        robotDrive.setDriveType(DriveType.ARCADE);
        robotArm.init();

    }
    //code to run once when the 'PLAY' button is triggered
    @Override
    public void start() { runtime.reset(); }
    //code to loop between PLAY and STOP buttons
    public void loop() {
        autoBrake = april.autoBrake(brakeDistance);
        robotDrive.drive(autoBrake);
        robotArm.loop();
    }
    @Override
    public void stop() {
    }
}
