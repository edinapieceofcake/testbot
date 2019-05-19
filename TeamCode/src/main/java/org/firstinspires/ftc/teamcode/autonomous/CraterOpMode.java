package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.enums.AutonomousStates;

@Autonomous(name="Crater", group="Autonomous")
@Disabled
public class CraterOpMode extends BaseAutoOpMode{
    private int distanceFromLeftMineral = (int)(DrivePerInch * 19.5);

    public void runOpMode(){

        //
        // get the robot setup and ready to run
        //
        AutonomousStates currentState = AutonomousStates.START;

        InitRobot();
        InitGyro();

        robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // hang the robot
        currentState = Latch();

        while (!isStarted()) {
            synchronized (this) {
                try {
                    // look for the mineral and tell us what the camera sees
                    LocateTFMineral();
                    telemetry.addData("Mineral Location", mineralLocation);
                    telemetry.addData("Last Recognition", LastRecognition);
                    telemetry.addData("Angle", GetImuAngle());
                    telemetry.addData("Latch Power: ", robot.getBackLift().getPower());
                    telemetry.addData("Flip Position", robot.getFrontFlip().getCurrentPosition());
                    telemetry.update();
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }

        //
        // Our state machine for what we do when we are landing from the crater.
        //
        while (opModeIsActive() && (currentState != AutonomousStates.MINED)) {
            switch (currentState) {
                case LATCHED:
                    currentState = Drop();
                    break;
                case DROPPED:
                    currentState = MoveLeftOffLatch();
                    break;
                case MOVED_OFF_LATCH:
                    currentState = MoveForward(driveForwardPosition);
                    break;
                case MOVED_FORWARD:
                    currentState = DriveToMineralOffLeftOffset(slideLeftPosition, slideRightPosition);
                    break;
                case AT_MINERAL:
                    currentState = PushMineral((int)(DrivePerInch * PushMineralDistance));
                    break;
                case MINERAL_PUSHED:
                    currentState = BackAwayFromMineral((int)(DrivePerInch * BackAwayFromMineralDistance));
                    break;
                case BACKED_AWAY_FROM_MINERAL:
                    currentState = MoveToMiddleAtMineral(slideLeftPosition, slideRightPosition);
                    break;
                case BACK_AT_MIDDLE:
                    currentState = Mine();
                    break;
            }
        }



    }
}
