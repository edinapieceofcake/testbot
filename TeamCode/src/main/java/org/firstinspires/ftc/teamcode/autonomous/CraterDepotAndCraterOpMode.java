package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.enums.AutonomousStates;

@Autonomous(name="CraterDepotCrater", group="Autonomous")
@Disabled
public class CraterDepotAndCraterOpMode extends BaseAutoOpMode{
    private int distanceFromLeftMineral = (int)(DrivePerInch * 19.5);

    public void runOpMode(){

        AutonomousStates currentState = AutonomousStates.START;

        InitRobot();

        robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        currentState = Latch();

        InitGyro();

        while (!isStarted()) {
            synchronized (this) {
                try {
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

        while (opModeIsActive() && (currentState != AutonomousStates.AT_CRATER)) {
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
                    currentState = MoveToLeftWall(distanceFromLeftMineral, slideLeftPosition + distanceFromLeftMineral,
                            slideLeftPosition + slideRightPosition + distanceFromLeftMineral, .9);
                    robot.getFrontFlip().setTargetPosition(800);
                    robot.getFrontFlip().setPower(.9);

                    while (robot.getFrontFlip().isBusy()) {
                        idle();
                    }

                    break;
                case AT_LEFT_WALL:
                    currentState = TurnTowardsDepotFromCrater();
                    mecanum.SlideRight2(.9,1000,this);
                    break;
                case TURNED_TOWARDS_CRATER:
                    currentState = MoveTowardsDepot();
                    break;
                case AT_DEPOT:
                    robot.getFrontFlip().setTargetPosition(1800);
                    robot.getFrontFlip().setPower(.9);

                    while (robot.getFrontFlip().isBusy()) {
                        idle();
                    }

                    currentState = AutonomousStates.DROPPED_MARKER; //DropMarker();

                    robot.getFrontFlip().setTargetPosition(800);
                    robot.getFrontFlip().setPower(.9);

                    while (robot.getFrontFlip().isBusy()) {
                        idle();
                    }

                    break;
                case DROPPED_MARKER:
                    currentState = TurnTowardsCraterFromDepot2();
                    break;
                case FACING_CRATER:
                    currentState = DriveTowardsCrater();
                    break;
            }
        }
    }
}
