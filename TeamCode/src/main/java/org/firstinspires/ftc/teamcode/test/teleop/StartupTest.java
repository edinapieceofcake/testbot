package org.firstinspires.ftc.teamcode.test.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Test: Startup", group = "Autonomous Test")
@Disabled
public class StartupTest extends OpMode {
    @Override
    public void internalPreInit() {
        super.internalPreInit();
        if (hardwareMap != null) {
            telemetry.addData("Hardware Map", "not null");
        } else {
            telemetry.addData("Hardware Map", "null");
        }

        telemetry.update();
    }

    public void init() {

    }

    public void loop() {

    }
}
