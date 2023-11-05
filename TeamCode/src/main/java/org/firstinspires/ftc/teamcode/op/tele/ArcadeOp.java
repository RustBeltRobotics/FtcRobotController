package org.firstinspires.ftc.teamcode.op.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.model.DriveType;

@TeleOp(name="Tele-Arcade", group="Iterative OpMode")
public class ArcadeOp extends TeleOpParent {

    @Override
    public DriveType getDriveType() {
        return DriveType.ARCADE;
    }
}
