package org.firstinspires.ftc.teamcode.autonomous;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.edinaftcrobotics.drivetrain.TelemetryMounts;
import com.edinaftcrobotics.navigation.PurePursuit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

import java.util.ArrayList;

@Autonomous(name="PursuitTest")
public class PursuitTest extends LinearOpMode {

    private PieceOfCake r;
    private Mecanum m;
    private TelemetryMounts tm;
    private PurePursuit p;

    @Override
    public void runOpMode() {
        r = new PieceOfCake();
        r.init(hardwareMap);
        m = new Mecanum(r.getFrontL(), r.getFrontR(), r.getBackL(), r.getBackR(), true, telemetry);
        m.StopAndResetMotors3();
        tm = new TelemetryMounts(2,4,1400,15.5);
        p = new PurePursuit(m, tm, 5);

//        Waiting for user to start opmode
        while(!opModeIsActive());

        ArrayList<double[]> path = new ArrayList<>();
        path.add(new double[]{0, 0});
        path.add(new double[]{-20, 0});
        path.add(new double[]{-20, 40});
        path.add(new double[]{20, 40});
        path.add(new double[]{20, 0});
        path.add(new double[]{0, 0});

        m.setCurrentPower(0.3);
        p.start();
        p.moveTo(path);
        while(p.isBusy()){
            telemetry.addLine(p.getTelemetry());
            telemetry.update();
        }
        p.shutDown();
    }

}
