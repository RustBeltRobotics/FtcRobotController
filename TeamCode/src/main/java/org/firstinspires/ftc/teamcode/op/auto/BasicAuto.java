package org.firstinspires.ftc.teamcode.op.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.controller.PIDController;
import org.firstinspires.ftc.teamcode.model.AutoCommands;
import org.firstinspires.ftc.teamcode.model.TargetPosition;
import org.firstinspires.ftc.teamcode.util.DebugLog;

@Config
@Autonomous(name = "BasicAuto", group = "Robot", preselectTeleOp = "Tele-Arcade")
public class BasicAuto extends LinearOpMode {

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // neverest 20 - PAT / RBR value
    static final double DRIVE_GEAR_REDUCTION = (15.0 / 20.0);     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    public static int TICKS_PER_PIXEL_OUTTAKE = 90;
    public static double DRIVE_SPEED = .25;
    public static double MAX_TURN_SPEED = 0.1;
    public static double TURN_GAIN = 0.005;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public static double HEADING_THRESHOLD = 2.0; // How close must the heading get to the target before moving to next step.
    public static int ARM_SCORING_POSITION = -354;

    public static double TURN_PID_P = 0.005;
    public static double TURN_PID_I = 0.01;
    public static double TURN_PID_D = 0.0005;

    private String startingPositionLetter = "F";
    private int startingPositionNumber = 2;

    private DcMotorEx left1 = null;
    private DcMotorEx left2 = null;
    private DcMotorEx right1 = null;
    private DcMotorEx right2 = null;
    private DcMotorEx arm1 = null;
    private DcMotorEx arm2 = null;
    private DcMotorEx intake1 = null;
    private DcMotorEx ext1 = null;
    private BHI260IMU imu = null;

    private PIDController pidRotate;

    private ElapsedTime elapsedTime = null;
    private int waitForStartTime = 0;

    private double positionX;
    private double positionY;
    private double targetHeading = 0;
    private double headingError = 0.0;
    private double turnSpeed = 0;

    private double globalAngle;
    private double angleCorrection;
    private double rotation;
    private Orientation lastAngles = new Orientation();

    @Override
    public void runOpMode() throws InterruptedException {
        initBot();
        waitForStart();
//		while (opModeIsActive()) {
//			runLoop();
//		}
        runAutoRoutine();
    }

    private void initBot() {
        targetHeading = 0;
        headingError = 0;
        positionX = 0;
        positionY = 0;

        left1 = hardwareMap.get(DcMotorEx.class, "L1");
        left2 = hardwareMap.get(DcMotorEx.class, "L2");
        right1 = hardwareMap.get(DcMotorEx.class, "R1");
        right2 = hardwareMap.get(DcMotorEx.class, "R2");

        arm1 = hardwareMap.get(DcMotorEx.class, "A1");
        arm2 = hardwareMap.get(DcMotorEx.class, "A2");
        intake1 = hardwareMap.get(DcMotorEx.class, "I1");
        ext1 = hardwareMap.get(DcMotorEx.class, "E1");

        left1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        ext1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        left1.setDirection(DcMotorEx.Direction.REVERSE);
        left2.setDirection(DcMotorEx.Direction.FORWARD);
        right1.setDirection(DcMotorEx.Direction.REVERSE);
        right2.setDirection(DcMotorEx.Direction.FORWARD);
        arm1.setDirection(DcMotorEx.Direction.FORWARD);
        arm2.setDirection(DcMotorEx.Direction.REVERSE);
        intake1.setDirection(DcMotorEx.Direction.FORWARD);
        ext1.setDirection(DcMotorEx.Direction.FORWARD);

        left1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        ext1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        left1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ext1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(TURN_PID_P, TURN_PID_I, TURN_PID_D);
//        pidRotate.setDebugLog(true);

        //setup FTC dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initPose(startingPositionLetter, startingPositionNumber);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw(); //zero out our rotation angle
        elapsedTime = new ElapsedTime();
    }

    private void runAutoRoutine() {
        //Sample auto routine - drive forward 8 inches, turn left 60 degrees, drop pixel, backup slightly to avoid hitting pixel
//        dumbDrive(DRIVE_SPEED, 8.0, 8.0, 30);
//        pidTurn(0.5, 60); //turn left 60 degrees
//        outtake(1); //drop pixel
//        dumbDrive(DRIVE_SPEED, -4.0, -4.0, 30);  //back up 4 inches to avoid driving over dropped pixel
//        pidTurn(0.5, -60); //turn right 60 degrees to straighten back out
//        dumbDrive(DRIVE_SPEED, 12.0, 12.0, 30);  //move forward a foot

        //Note: approx. 21 inches from back of bot to edge of first pre-loaded pixel to drop
        //drive forward 24 inches, turn left 20 degrees, outtake 1 pixel, backup 2 inches, turn right 110 degrees
        AutoCommands autoCommands = new AutoCommands(List.of("D 26.0", "T 20.0", "O 1", "D -2.0", "T -110.0"));
//        executeAutoCommands(autoCommands);
/*
        //15x3 = 45 degree left total
        pidTurn(MAX_TURN_SPEED, 15);
        sleep(200);
        pidTurn(MAX_TURN_SPEED, 15);
        sleep(200);
        pidTurn(MAX_TURN_SPEED, 15);
        sleep(3000);
        pidTurn(MAX_TURN_SPEED, 90);
        sleep(3000);
        //should return back to starting position
        pidTurn(MAX_TURN_SPEED, -135);
        sleep(3000);
*/

//        DebugLog.log("Turning right 90 degrees");
//        pidTurn(0.5, -90); //turn right 90 degrees
//        sleep(1200);
//        DebugLog.log("Turning left 90 degrees");
//        pidTurn(0.5, 90); //turn left 90 degrees



//        dumbDrive(DRIVE_SPEED, 12.0, 12.0, 30);

//        DebugLog.log("Turning left 45 degrees");
//        pidTurn(0.5, 45); //turn left 45 degrees
//        sleep(1200);
//        DebugLog.log("Turning left 90 degrees");
//        pidTurn(0.5, 90); //turn left 90 degrees
//        sleep(1200);

        //working!  Red Alliance F2
//		setStartingPosition(-36.0, -63.5);
//		smartDrive(-36.0, -36.5, true);
//		turnToHeading(0.5, -45);//turn right 45 degrees
//		turnToHeading(0.5, 45);//turn left 45 degrees

        //Working! Blue Alliance A2
//        setStartingPosition(-36.0, 63.5);
//        smartDrive(-36.0, 36.5, false);
//        turnToHeading(0.1, 45);//turn left 45 degrees
//        turnToHeading(0.5, -45);//turn right 45 degrees

//		turnToHeading(0.5, -45); //turn right 45 degrees
//		turnToHeading(0.5, 45); //turn left 45 degrees
//		dumbDrive(DRIVE_SPEED, 24.0, 24.0, 30);
//		smartDrive(-36.0, -36.5); //Red F2 starting POS = -36 (X), -63.5 (Y)
//		smartDrive(-24.0, -30.5);
//		smartDrive(24, 30.5);
//		smartDrive(-36.0, 0);
//		smartDrive(36.0, 0);
        double currentMs = elapsedTime.milliseconds();
//		outputTelemetry();
        double targetMs = currentMs + 5000;
        while (opModeIsActive()) {
            outputTelemetry();
        }
    }

    private void executeAutoCommands(AutoCommands autoCommands) {
        //T <Double> = Turn X degrees
        //D <Double> = Drive straight X inches
        //S <Long> = Sleep/stop for X milliseconds
        //O <Integer> = Outtake X pixels
        //RT = Determine randomization target position
        //P1 = score Pixel target 1 - drop pixel on target spike mark
        //P2 = score Pixel target 2 - place pixel on target slot on alliance backdrop
        for (String command : autoCommands.getCommands()) {
            if (command.startsWith("T")) {
                double turnDegrees = Double.parseDouble(command.split(" ")[1]);
                pidTurn(MAX_TURN_SPEED, turnDegrees);
            } else if (command.startsWith("D")) {
                double driveInches = Double.parseDouble(command.split(" ")[1]);
                dumbDrive(DRIVE_SPEED, driveInches, driveInches, 5);
            } else if (command.startsWith("S")) {
                long sleepMs = Long.parseLong(command.split(" ")[1]);
                sleep(sleepMs);
            } else if (command.startsWith("O")) {
                int numPixels = Integer.parseInt(command.split(" ")[1]);
                outtake(numPixels);
            } else if (command.startsWith("RT")) {
                double currentMs = elapsedTime.milliseconds();
                double targetMs = currentMs + 5000;
                boolean randomizationTargetFound = false;
                do {
                    currentMs = elapsedTime.milliseconds();
                    //TODO: test/implement implement
//                    randomizationTargetFound = RTDP.getDetectedPosition() != null;
                } while (opModeIsActive() && currentMs < targetMs && !randomizationTargetFound);
            } else if (command.startsWith("P1")) {
                //TODO: implement
                //scoreSpikeMarkPixel(RTDP.getDetectedPosition());
            } else if (command.startsWith("P2")) {
                //TODO: implement
            } else {
                throw new IllegalArgumentException("Unexpected autonomous string command: " + command);
            }
        }
    }

    private void scoreSpikeMarkPixel(TargetPosition targetPosition) {
        DebugLog.log("Scoring spike mark pixel - location %s", targetPosition.toString());
        if (targetPosition == null) {
            return; //we didn't find a target - nothing to do
        }

        switch (targetPosition) {
            case LEFT:
                //TODO turn left 45 degrees, drive forward until over spike mark, outtake 1 pixel
                break;
            case CENTER:
                //TODO turn left 15 degrees, drive forward until over spike mark, outtake 1 pixel
                break;
            case RIGHT:
                //TODO turn right 45 degrees, drive forward until over spike mark, outtake 1 pixel
                break;
        }

        //TODO: parameterize / determine if we're driving next to the wall (A for Blue, F for Red)
        // or through center truss (C for Blue, D for Red)
        //TODO: introduce a sleep/delay or use a distance sensor to avoid driving into alliance robot

        //Note: alternatively we may be able to use heuristic:
        // If Start row is 4, back up to wall before driving straight towards board, turn 90 degrees, straight, turn 90 degrees to in front of board position
        // If Start row is 2, take the longer way through truss
    }

    private void outputTelemetry() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Heading", " %.1f", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Heading (Normalized)", " %.1f", AngleUnit.normalizeDegrees(orientation.getYaw(AngleUnit.DEGREES)));
        telemetry.addData("Encoders", "Left %d  Right %d", left1.getCurrentPosition(), right1.getCurrentPosition());

        telemetry.update();
    }

    private void raiseArmToScoringPosition() {
        arm1.setTargetPosition(ARM_SCORING_POSITION);
        arm1.setPower(.5);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elapsedTime.reset();
        double raiseTimeoutSeconds = 5.0;

        while (opModeIsActive() && elapsedTime.seconds() < raiseTimeoutSeconds && arm1.isBusy()) {
            arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm2.setPower(.5);
            DebugLog.log("Raising arm to scoring position %d, current pos %d", arm1.getCurrentPosition(), ARM_SCORING_POSITION);
        }

        arm1.setPower(0);
        arm2.setPower(0);
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void pidTurn(double maxTurnSpeed, double degrees) {
        DebugLog.log("pidTurn() - %.1f degrees", degrees);
        resetAngle();

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setPID(TURN_PID_P, TURN_PID_I, TURN_PID_D);
        pidRotate.setSetPoint(degrees);
//        pidRotate.setInputRange(0, degrees);
//        pidRotate.setInputRange(0, 180);
//        pidRotate.setOutputRange(0, maxTurnSpeed);
        pidRotate.setTolerance(1);
//        pidRotate.enable();

        double power = maxTurnSpeed;

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                DebugLog.log("Turn - get off zero!  current heading = %.1f, calculated power = %.1f", getAngle(), power);
                left1.setPower(power);
                left2.setPower(power);
                right1.setPower(-power);
                right2.setPower(-power);
                sleep(100);
            }

            do {
                double currentDegrees = getAngle();
                power = pidRotate.calculate(currentDegrees); // power will be negative on right turn.
                DebugLog.log("Turning right!  current heading = %.1f, calculated power = %.1f", currentDegrees, power);
                left1.setPower(-power);
                left2.setPower(-power);
                right1.setPower(power);
                right2.setPower(power);
            } while (opModeIsActive() && !pidRotate.atSetPoint());
        } else {
            // left turn.
            do {
                double currentDegrees = getAngle();
                power = pidRotate.calculate(currentDegrees); // power will be positive on left turn.
                DebugLog.log("Turning left!  current heading = %.1f, calculated power = %.1f", currentDegrees, power);
                left1.setPower(-power);
                left2.setPower(-power);
                right1.setPower(power);
                right2.setPower(power);
            } while (opModeIsActive() && !pidRotate.atSetPoint());
        }

        // turn the motors off.
        left1.setPower(0);
        left2.setPower(0);
        right1.setPower(0);
        right2.setPower(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
        DebugLog.log("completing pidTurn - rotation = %.1f, lastAngles = %s", rotation, lastAngles);
    }

    private void moveRobot(double x, double yaw) {
        // Calculate left and right wheel powers.
        double leftPower = x - yaw;
        double rightPower = x + yaw;

        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            DebugLog.log("max drive motor power exceeded! (%.1f) reducing to: %.1f", max, Math.abs(leftPower / max));
            leftPower /= max;
            rightPower /= max;
        }

        DebugLog.log("moveRobot() Left: (%.1f) Right: %.1f", leftPower, rightPower);
        // Send powers to the wheels.
        left1.setPower(leftPower);
        left2.setPower(leftPower);
        right1.setPower(rightPower);
        right2.setPower(rightPower);
    }

    private void dumbDrive(double speed,
                           double leftInches, double rightInches,
                           double timeoutS) {

        int newLeft1Target;
        int newLeft2Target;
        int newRight1Target;
        int newRight2Target;

        DebugLog.log("Dumbdrive start - L1 Pos %7d, R1 Pos %7d", left1.getCurrentPosition(), right1.getCurrentPosition());

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeft1Target = left1.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newLeft2Target = left2.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRight1Target = right1.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newRight2Target = right2.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            left1.setTargetPosition(newLeft1Target);
            left2.setTargetPosition(newLeft2Target);
            right1.setTargetPosition(newRight1Target);
            right2.setTargetPosition(newRight2Target);

            // Turn On RUN_TO_POSITION
            left1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            left2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            right1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            right2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            elapsedTime.reset();
            //TODO: switch this to use PID control so we don't overshoot our target
            left1.setPower(Math.abs(speed));
            left2.setPower(Math.abs(speed));
            right1.setPower(Math.abs(speed));
            right2.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (elapsedTime.seconds() < timeoutS) &&
                    (left1.isBusy() && right1.isBusy())) {
                // Display it for the driver.
                telemetry.setMsTransmissionInterval(20);
                telemetry.addData("Running to", " %7d :%7d", newLeft1Target, newRight1Target);
                telemetry.addData("Currently at", " at %7d :%7d", left1.getCurrentPosition(), right1.getCurrentPosition());
                telemetry.addData("X", positionX);
                telemetry.addData("Y", positionY);
                telemetry.addData("Heading err", headingError);
                //sleep(20);
                telemetry.update();
            }

            //TODO: update current position

            telemetry.setMsTransmissionInterval(250);  //restore original default telemetry transmission interval
            left1.setPower(0);
            left2.setPower(0);
            right1.setPower(0);
            right2.setPower(0);

            left1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            left2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            right1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            right2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    private void initPose(String startingPositionY, int startingPositionX) {
        double[] arrayOfPossibilities = {-60.0, -36.0, -12.0, 12.0, 36.0, 60.0};
        double A = 28.0;
        double F = -28.0;
        double YstartPos = 0;

        switch (startingPositionY.substring(0, 1)) {
            case "A":
                YstartPos = 63.5;
                break;
            case "F":
                YstartPos = -63.5;
                break;
        }

        positionY = YstartPos;
        positionX = arrayOfPossibilities[startingPositionX - 1];
    }

    private void smartDrive(double X, double Y, boolean isRedAlliance) {
        //original
//		double triX = positionX - X;
//		double triY = positionY - Y;
        //SWAP X AND Y since our coordinate system is different
//		double triY = positionX - X;
//		double triX = positionY - Y;
        double triX = X - positionX;
        double triY = Y - positionY;

//		double headingIWant = -Math.toDegrees(Math.atan(triX/triY));
        double radians = Math.atan2(triY, triX);
        double headingIWant = Math.toDegrees(radians);  //yes, Y is supposed to come first as atan2 arg - check javadocs
        double range = Math.hypot(triX, triY);
        double adjustedHeading = adjustAngleForAllianceStartingPosition(headingIWant, isRedAlliance);
        DebugLog.log("Driving from (%.1f, %.1f) to (%.1f, %.1f) with %.1f raw degree turn (%.1f adjusted) with distance %.1f inches %n", positionX, positionY, X, Y, headingIWant, adjustedHeading, range);
        pidTurn(1, adjustedHeading);

//		double range = Math.sqrt((Math.pow(triX, 2) + (Math.pow(triY, 2))));

        //27.0 inches target distance - encoder ticks measured = 820.0, 32.065 ticks per inch * 27 inches = 866.155

        dumbDrive(DRIVE_SPEED, range, range, 30);
    }

    private double adjustAngleForAllianceStartingPosition(double angle, boolean isRedAlliance) {
        if (isRedAlliance) {
            return angle;
        } else {
            return angle + 180;
        }
    }

    private void setStartingPosition(double x, double y) {
        positionX = x;
        positionY = y;
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        imu.resetYaw();
        DebugLog.log("resetAngle() - lastAngles = %s, new current IMU angle = %.1f", lastAngles, imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        DebugLog.log("getAngle() 1 - currentAngle = %.1f, lastAngle = %.1f, deltaAngle = %.1f, globalAngle = %.1f", angles.firstAngle, lastAngles.firstAngle, deltaAngle, globalAngle);

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        DebugLog.log("getAngle() 2 - deltaAngle = %.1f, globalAngle = %.1f", deltaAngle, globalAngle);

        return globalAngle;
    }

    private void outtake(int numPixels) {
        int target = 0;
        //figure out how many pixels we want to drop
        if (numPixels == 1) {
            target = intake1.getCurrentPosition() + TICKS_PER_PIXEL_OUTTAKE;
        } else if (numPixels == 2) {
            target = intake1.getCurrentPosition() + (2 * TICKS_PER_PIXEL_OUTTAKE);
        } else {
            telemetry.addData("outtake() problem:", numPixels + " is not 1 or 2. try again stupid");
        }
        intake1.setTargetPosition(target);
        intake1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intake1.setPower(1.0);

        //run until we reach target position
        while (opModeIsActive() && intake1.isBusy()) {
            DebugLog.log("Outtaking - current position = %d, target position = %d", intake1.getCurrentPosition(), target);
        }

        intake1.setPower(0);
        intake1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

}
