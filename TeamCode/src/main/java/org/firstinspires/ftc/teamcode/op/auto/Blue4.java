package org.firstinspires.ftc.teamcode.op.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.model.Alliance;

//Closer to scoring backdrop, further from audience
@Autonomous(name = "Blue4", group = "Robot", preselectTeleOp = "Tele-Arcade")
public class Blue4 extends CompetitionAuto {

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    protected String getStartingPosition() {
        return "A4";
    }
}
