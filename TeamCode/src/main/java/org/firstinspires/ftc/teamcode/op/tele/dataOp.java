/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.op.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="dataOp", group="Iterative OpMode")
//@Disabled
public class dataOp extends OpMode
{
    // Declare OpMode members.
    private Gamepad gamepad1;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx left1 = null;
    private DcMotorEx left2 = null;
    //define right motors 1 and 2
    private DcMotorEx right1 = null;
    private DcMotorEx right2 = null;
    private DcMotorEx arm1 = null;
    private DcMotorEx arm2 = null;
    private DcMotorEx intake1 = null;
    private IMU imu = null;
    YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
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

        left1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        left2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        right1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        right2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        left1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        left1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        arm1 = hardwareMap.get(DcMotorEx.class, "A1");
        arm2 = hardwareMap.get(DcMotorEx.class, "A2");

        arm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        arm1.setDirection(DcMotorEx.Direction.FORWARD);
        arm2.setDirection(DcMotorEx.Direction.REVERSE);

        arm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        arm2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        //intake
        intake1 = hardwareMap.get(DcMotorEx.class, "I1");

        intake1.setDirection(DcMotorEx.Direction.FORWARD);

        intake1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }
    @Override
    public void init_loop() {
        telemetry.addData("! Press","A to reset encoders");
        telemetry.addData("L1 encoder",left1.getCurrentPosition());
        telemetry.addData("L2 encoder",left2.getCurrentPosition());
        telemetry.addData("R1 encoder",right1.getCurrentPosition());
        telemetry.addData("R2 encoder",right2.getCurrentPosition());
        telemetry.addData("arm1 encoder",arm1.getCurrentPosition());
        telemetry.addData("arm2 encoder",arm2.getCurrentPosition());
        telemetry.addData("intake encoder",intake1.getCurrentPosition());
        telemetry.addData("heading", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.update();

        if ( gamepad1.a == true ) {
            left1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            left2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            right1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            right2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            arm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            arm2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            intake1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
