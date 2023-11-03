package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name="Competition", group="Iterative OpMode")
//@Disabled
public class RobotMain extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private RobotArm robotArm = null;
    private RobotDrive robotDrive = null;
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        robotArm = new RobotArm(hardwareMap, runtime, gamepad2, telemetry);
        robotDrive = new RobotDrive(hardwareMap, runtime, gamepad1, telemetry);

        telemetry.addData("Power Left val:", gamepad1.toString());
        telemetry.addData("Status", "Initialized");

        robotDrive.init();
        robotArm.init();
    }
    //code to run once when the 'PLAY' button is triggered
    @Override
    public void start() { runtime.reset(); }
    //code to loop between PLAY and STOP buttons
    public void loop() {
        robotDrive.loop();
        robotArm.loop();
    }
    @Override
    public void stop() {
    }
}
