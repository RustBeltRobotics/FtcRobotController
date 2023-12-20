package org.firstinspires.ftc.teamcode.op.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.model.DriveType;

@TeleOp(name="Tele-Tank", group="Iterative OpMode")
public class TankOp extends TeleOpParent {
    @Override
    public DriveType getDriveType() {
        return DriveType.TANK;
    }
}
