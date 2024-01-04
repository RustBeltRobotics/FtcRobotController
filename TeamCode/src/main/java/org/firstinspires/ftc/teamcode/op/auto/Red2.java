package org.firstinspires.ftc.teamcode.op.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.model.Alliance;

//Closer to audience, further from scoring backdrop
@Autonomous(name = "Red2", group = "Robot", preselectTeleOp = "Tele-Arcade")
public class Red2 extends CompetitionAuto {

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    protected String getStartingPosition() {
        return "F2";
    }
}
