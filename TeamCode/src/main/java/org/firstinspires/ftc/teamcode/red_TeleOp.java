package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "01 Red Alliance (TeleOp)", group = "Robot")
//@Disabled

public class red_TeleOp extends base_TeleOp {

    @Override
    public void runOpMode() {
        super.setAllianceColor("RED");
        super.runOpMode();
    }

}