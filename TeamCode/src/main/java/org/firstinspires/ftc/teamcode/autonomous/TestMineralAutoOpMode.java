package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.enums.AutonomousStates;

@Autonomous(name="Test Mineral Detection", group="Autonomous")
@Disabled
public class TestMineralAutoOpMode extends BaseAutoOpMode {
    private int distanceFromLeftMineral = DrivePerInch * 21;

    public void runOpMode() {
        AutonomousStates currentState = AutonomousStates.START;

        InitRobot();
        InitGyro();

        currentState = Latch();

        while (!isStarted()) {
            synchronized (this) {
                try {
                    LocateTFMineral2();
                    telemetry.addData("Mineral Location", mineralLocation);
                    telemetry.addData("Last Recognition", LastRecognition);
                    telemetry.update();
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }

        Drop();

        while (opModeIsActive()) {
            telemetry.addData("Mineral Location", mineralLocation);
            telemetry.update();
        }

        ShutdownTFOD();
    }
}
