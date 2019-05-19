package org.firstinspires.ftc.teamcode.autonomous;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.edinaftcrobotics.navigation.TurnOMatic;
import com.edinaftcrobotics.navigation.TurnOMatic2;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.enums.AutonomousStates;
import org.firstinspires.ftc.teamcode.enums.MineralLocation;
import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

import java.util.List;

abstract class BaseAutoOpMode extends LinearOpMode {
    protected int DrivePerInch = (int)(1120 / 18.85);
    private int FlatFlip = 1800;
    private int SlideOffLatchDistance = 275;
    private int slideCenterPosition = 250;
    private int Turn90 = 1225;
    private int Turn45 = Turn90/2;
    private ElapsedTime watch = new ElapsedTime();
    protected double PushMineralDistance = 6;
    protected double BackAwayFromMineralDistance = 5.5;
    protected int slideRightPosition = DrivePerInch * 23;
    protected int slideLeftPosition = DrivePerInch * 23;
    protected int driveForwardPosition = (int)(DrivePerInch * 19);

    private static final String VUFORIA_KEY = "ASA9XvT/////AAABmUnq30r9sU3Nmf/+RS+Xx0CHgJj/JtD5ycahnuM/0B2SFvbMRPIZCbLi4LeOkfse9Dymor5W7vNMYI+vmqVx9kpEaKE8VM7cFMUb/T1LLwlCPdX9QKOruzTcRdlYswR7ULh4K11GuFZDO/45pSks+Nf25kT5cnV+IN3TsscA0o7I6XPIeUoAJJPsjw+AycsmRk2uffr3Bnupexr93iRfHylniqP+ss4cRcT1lOqS5Zhh7FQaoelR58qL/RUorGpknjy9ufCn9ervc6Mz01u3ZkM/EOa5wUPT8bDzPZ6nMDaadqumorT5Py+GtJSUosUgz4Gd3iR++fdEk6faFZq3L9xfBSagNykwhiyYx+oqwVqe";
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    protected MineralLocation mineralLocation = MineralLocation.RIGHT;
    private VuforiaLocalizer vuforia;

    protected PieceOfCake robot = new PieceOfCake();
    protected Mecanum mecanum = null;

    protected BNO055IMU imu = null;
    protected TFObjectDetector tfod;
    protected Recognition LastRecognition = null;

    //
    // This is our init section.  We have all the code here that we will use to init and setup the robot for
    // autonomous.  The main parts are:
    //
    //  initRobot - Setup the drive and video
    //  initVuforia - Used to setup the camera for mineral detection
    //  initTFod - Used to setup the TensorFlow for mineral detection
    //  initGyro - Used to setup the gyro for use when we want to turn
    //
    protected void InitRobot() {
        robot.init(hardwareMap);

        mecanum = new Mecanum(robot.getFrontL(), robot.getFrontR(), robot.getBackL(), robot.getBackR(), true, telemetry);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }
    }

    //
    // Init the gyro for turns and general heading information.  We get the imu and set it up for
    // degrees.  We then wait for it to get ready
    //
    protected void InitGyro() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated()) {
            telemetry.addData("I am trying to init the gyro", "so stop moving the lander or robot Jack!");
            telemetry.update();
        }
    }

    protected double GetImuAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle;
    }

    //
    // Init the tensor flow for mineral detection.  We load the model and set the mineral location to the one
    // on the right.  You can read up on tensorflow here https://en.wikipedia.org/wiki/TensorFlow
    //
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        mineralLocation = MineralLocation.RIGHT;
    }

    //
    // Init the camera so we can use to to find the right mineral to knock off.
    //
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    //
    // Our mineral logic is different than what was provided by FIRST.  Our camera cannot see all
    // three minerals, so we had to change the logic to look at all the things it found.
    // After some debugging, we determined the right top and bottom range of the minerals from the
    // view of the camera We then figured out the left positions and used that with the top,
    // bottom, and label to find the mineral location.  If we didn't see a gold mineral at all,
    // we then picked the right one
    //
    public void LocateTFMineral() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if ((updatedRecognitions != null) && (updatedRecognitions.size() > 0)) {
                mineralLocation = MineralLocation.RIGHT;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData("Object", recognition);
                    //
                    // We are looking for a gold mineral that is between 520 and 730 units from the
                    // phone.  This number can be changed based on actual field testing at the
                    // competition
                    //
                    if ((recognition.getLabel().equals(LABEL_GOLD_MINERAL)) &&
                            (recognition.getTop() > 520) && (recognition.getBottom() < 730)) {
                        //
                        // Now we check the left position of the mineral to see if is the left or
                        // middle one.  This number can be changed based on actual field testing
                        // at the competition
                        //
                        int goldMineralX = (int) recognition.getLeft();
                        if (goldMineralX < 430) {
                            mineralLocation = MineralLocation.LEFT;
                        } else if ((goldMineralX >= 430)  && (goldMineralX <= 650)) {
                            mineralLocation = MineralLocation.MIDDLE;
                        } else {
                            mineralLocation = MineralLocation.RIGHT;
                        }

                        LastRecognition = recognition;

                        break;
                    }
                }
            } else {
                telemetry.addData("Nothing New", "Detected");
            }
        } else {
            telemetry.addData("No", "TFOD");
        }
    }

    public void LocateTFMineral2() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if ((updatedRecognitions != null) && (updatedRecognitions.size() > 0)) {
                mineralLocation = MineralLocation.RIGHT;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData("Object", recognition);
                    //
                    // We are looking for a gold mineral that is between 520 and 730 units from the
                    // phone.  This number can be changed based on actual field testing at the
                    // competition
                    //
                    /*if ((recognition.getLabel().equals(LABEL_GOLD_MINERAL)) &&
                            (recognition.getTop() > 520) && (recognition.getBottom() < 730)) */{
                        //
                        // Now we check the left position of the mineral to see if is the left or
                        // middle one.  This number can be changed based on actual field testing
                        // at the competition
                        //
                        int goldMineralX = (int) recognition.getLeft();
                        if (goldMineralX < 430) {
                            mineralLocation = MineralLocation.LEFT;
                        } else if ((goldMineralX >= 430)  && (goldMineralX <= 650)) {
                            mineralLocation = MineralLocation.MIDDLE;
                        } else {
                            mineralLocation = MineralLocation.RIGHT;
                        }

                        LastRecognition = recognition;

                        break;
                    }
                }
            } else {
                telemetry.addData("Nothing New", "Detected");
            }
        } else {
            telemetry.addData("No", "TFOD");
        }
    }

    public void ShutdownTFOD() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    //
    // The following section is our many different states for our state machine which is used
    // during autonomous.  Each state returns a finished state which the autonomous state machine
    // uses to figure out what to do next.  Each state can be used multiple times in the same
    // machine if we want to.  That is why we went with a state machine.  It was easy to
    // build, modify, and test.  To learn more about state machines, visit
    // https://en.wikipedia.org/wiki/Finite-state_machine
    //
    // Our states are:
    //  Latch - Power the lift so it will hang on the lander
    //  MoveToLeftWall - slide from a mineral location to the left wall
    //  Drop - drops the robot from the lander
    //  MoveLeftOffLatch - we slide left to get detached from the hook and then move back to
    //      straighten out
    //  MoveForwardAndSlideBackToCenter - we move forwards and center ourselves in front of the
    //      lander
    //  DriveToMineral - drive to mineral from center position
    //  DriveToMineralOffLeftOffset - drive to mineral right after we drop
    //  PushMineral - we use TenserFlow to knock off the mineral
    //  BackAwayFromMineral - we back up from pushing the mineral off
    //  ExtendArm - we extend our arm into the crater
    //  DropMarker - we extend our arm out to drop the marker
    //  TurnLeftTowardsCrater2 - we turn left 135 degrees towards the crater using RUN_WITH_ENCODER
    //      vs using RUN_TO_POSITION
    //  MoveTowardsDepot - move off the wall a little and move towards the depot
    //  TurnTowardsCraterFromDepot2 - we turn 180 degrees towards the crater using the imu
    //  DriveTowardsCrater - we drive towards the crater to extend the arm
    //  MoveToMiddleAtMineral - after we knock of the mineral at crater we slide back to the middle
    //      so we can mine
    //  Mine - we go into the crater and try to grab minerals.
    //
    public AutonomousStates Latch () {
        double currentPower = .12;
        boolean aPressed = false;
        boolean bPressed = false;

        // loop while they adjust the power to get the robot to hang properly
        while (!gamepad2.x) {
            robot.getBackLift().setPower(-currentPower);
            robot.getFrontLift().setPower(currentPower);

            if (gamepad2.a) {
                aPressed = true;
            }

            if (!gamepad2.a && aPressed) {
                aPressed = false;
                currentPower -= .01;
                if (currentPower < .1) {
                    // don't let them go below 10%
                    currentPower = .1;
                }
            }

            if (gamepad2.b) {
                bPressed = true;
            }

            if (!gamepad2.b && bPressed) {
                bPressed = false;
                currentPower += .01;
                if (currentPower > .2) {
                    // don't let them go above 20%
                    currentPower = .2;
                }
            }

            // display the status on the screen
            telemetry.addData("Current Power", currentPower);
            telemetry.addData("Alex, press Gamepad2 A", " to decrease power");
            telemetry.addData("Alex, press Gamepad2 B", " to increase power");
            telemetry.addData("Alex, Press X to Exit", " and power will be locked in.");
            telemetry.update();
        }

        return AutonomousStates.LATCHED;
    }

    public AutonomousStates MoveToLeftWall(int distanceFromLeftMineral,
                                           int distanceFromCenterMineral,
                                           int distanceFromRightMineral,
                                           double power) {
        // Based on the mineral location, move the right distance to get to the left wall.
        if (mineralLocation == MineralLocation.RIGHT) {
            mecanum.SlideLeft2(power, distanceFromRightMineral, this);
        } else if (mineralLocation == MineralLocation.LEFT) {
            mecanum.SlideLeft2(power, distanceFromLeftMineral, this);
        } else if (mineralLocation == MineralLocation.MIDDLE) {
            mecanum.SlideLeft2(power, distanceFromCenterMineral, this);
        }

        return AutonomousStates.AT_LEFT_WALL;
    }

    public AutonomousStates Drop() {
        // lock the slide
        robot.getSlide().setPower(.1);
        // flip the arm down so it doesn't get hit
        robot.getTopFlip().setPosition(1);
        robot.getFrontFlip().setTargetPosition(FlatFlip);
        robot.getFrontFlip().setPower(.7);

        while (robot.getFrontFlip().isBusy() && opModeIsActive()) {
            idle();
        }

        // land the robot by turning off the motors that made us latch
        robot.getBackLift().setPower(0);
        robot.getFrontLift().setPower(0);

        watch.reset();
        while ((watch.milliseconds() < 1000)  && opModeIsActive()){
            idle();
        }

        // unhook from the lander
        robot.getBackLift().setPower(.3);
        robot.getFrontLift().setPower(-.3);
        watch.reset();
        while ((watch.milliseconds() < 400)  && opModeIsActive()){
            idle();
        }

        robot.getBackLift().setPower(0);
        robot.getFrontLift().setPower(0);

        return AutonomousStates.DROPPED;
    }

    public AutonomousStates MoveLeftOffLatch() {
        // slide a little to the left so we can be outside the hook
        robot.getTopFlip().setPosition(1);

        mecanum.SlideLeft2(.5, SlideOffLatchDistance, this);

        watch.reset();
        // move backwards to line up against the lander
        mecanum.Move(-.3, -.3);
        while ((watch.milliseconds() < 200)  && opModeIsActive()) {
            idle();
        }

        mecanum.Stop();

        // turn if we are not straight
        if ((GetImuAngle()) < 0) {
            mecanum.TurnRight(0.3, 20, this);
        }

        return AutonomousStates.MOVED_OFF_LATCH;
    }
    
    public AutonomousStates MoveForward(int forwardDistance) {
        // lock the slide so it doesn't move around
        robot.getSlide().setPower(.5);

        // move forward whatever distance we are told
        mecanum.MoveForward2(.7, forwardDistance, this);

        robot.getSlide().setPower(0);
        return AutonomousStates.MOVED_FORWARD;
    }

    public AutonomousStates MoveForwardAndSlideBackToCenter(int forwardDistance) {
        // lock the slide so it doesn't move around
        robot.getSlide().setPower(.5);

        // Move forward and then move right towards the center
        mecanum.MoveForward2(.6, forwardDistance, this);
        mecanum.SlideRight2(.7, SlideOffLatchDistance, this);

        robot.getSlide().setPower(0);

        return AutonomousStates.MOVED_BACK_TO_CENTER;
    }

    public AutonomousStates DriveToMineral (int slideLeftDistance, int slideRightDistance) {
        // depending on the mineral location, move to the right spot to get ready knock it off
        // we think we are already at the center one
        if (mineralLocation == MineralLocation.LEFT) {
            mecanum.SlideLeft2(.5, slideLeftDistance, this);
        } else if (mineralLocation == MineralLocation.RIGHT) {
            mecanum.SlideRight2(.5, slideRightDistance, this);
        }

        return AutonomousStates.AT_MINERAL;
    }

    public AutonomousStates DriveToMineralOffLeftOffset(int slideLeftDistance,
                                                        int slideRightDistance) {
        // depending on the mineral location, move to the right spot to get ready to knock it off
        // difference betweent his and the other one is that we will also move to the center
        robot.getSlide().setPower(.1);

        if (mineralLocation == MineralLocation.LEFT) {
            mecanum.SlideLeft2(.5, slideLeftDistance - SlideOffLatchDistance, this);
        } else if (mineralLocation == MineralLocation.RIGHT) {
            mecanum.SlideRight2(.5, slideRightDistance + SlideOffLatchDistance, this);
        } else {
            mecanum.SlideRight2(.5, slideCenterPosition, this);
        }

        robot.getSlide().setPower(0);

        return AutonomousStates.AT_MINERAL;
    }

    public AutonomousStates PushMineral (int pushDistance) {
        // lock the slide and drive foward to knock the mineral off
        robot.getSlide().setPower(.1);
        mecanum.MoveForward2(.7, pushDistance, this);
        robot.getSlide().setPower(0);

        return AutonomousStates.MINERAL_PUSHED;
    }

    public AutonomousStates BackAwayFromMineral(int backDistance) {
        // lock the slide and back away from the mineral
        robot.getSlide().setPower(.1);
        mecanum.MoveBackwards2(.7, backDistance, this);
        robot.getSlide().setPower(0);

        return AutonomousStates.BACKED_AWAY_FROM_MINERAL;
    }

    public AutonomousStates ExtendArm() {
        // stick the arm out for things like crater parking
        watch.reset();

        robot.getFrontFlip().setTargetPosition(FlatFlip);
        robot.getFrontFlip().setPower(.7);

        // slide arm out
        robot.getSlide().setPower(-1);
        // run for 1000 milliseconds
        while (watch.milliseconds() < 1500) {
            idle();
        }

        robot.getSlide().setPower(0);

        return AutonomousStates.ARM_EXTENDED;
    }

    public AutonomousStates DropMarker () {
        watch.reset();
        // slide arm out
        robot.getSlide().setPower(-1);
        // run for 1000 milliseconds
        while ((watch.milliseconds() < 1000)  && opModeIsActive()) {
            idle();
        }

        robot.getSlide().setPower(0);

        // spin the intake to dump marker
        watch.reset();
        robot.getIntake().setPower(-1);
        while ((watch.milliseconds() < 1750)  && opModeIsActive()) {
            idle();
        }

        robot.getIntake().setPower(0);

        // move slide back in
        watch.reset();
        robot.getSlide().setPower(1);
        // run for 1000 milliseconds
        while ((watch.milliseconds() < 1000)  && opModeIsActive()) {
            idle();
        }

        robot.getSlide().setPower(0);

        return AutonomousStates.DROPPED_MARKER;
    }

    public AutonomousStates TurnTowardsDepotFromCrater() {
        TurnOMatic2 turner = new TurnOMatic2(imu, mecanum, telemetry, 135, this);
        turner.Turn(.05, 3000);

        return AutonomousStates.TURNED_TOWARDS_CRATER;
    }

    public AutonomousStates TurnLeftTowardsCrater2() {
        // turn us left towards teh crater and get us close to the wall
        // we will turn 135 degrees
        mecanum.TurnLeft(.5, Turn45 + Turn90, this);

        mecanum.SlideRight2(.5, DrivePerInch * 15, this);

        return AutonomousStates.TURNED_TOWARDS_CRATER;
    }

    public AutonomousStates MoveTowardsDepot() {
        // move off the wall a little and move towards the depot
        mecanum.SlideLeft2(.7,100,this);

        mecanum.MoveForward2(.7, DrivePerInch * 20, this);

        return AutonomousStates.AT_DEPOT;
    }

    public AutonomousStates TurnTowardsCraterFromDepot2() {
        TurnOMatic2 turner = new TurnOMatic2(imu, mecanum, telemetry, -45, this);
        turner.Turn(.03, 3000);

        return AutonomousStates.FACING_CRATER;
    }

    public AutonomousStates DriveTowardsCrater(){
        // drive towards the crater
        mecanum.MoveForward2(.7, DrivePerInch * 20,this);

        return AutonomousStates.AT_CRATER;
    }

    public AutonomousStates MoveToMiddleAtMineral(int slideLeftDistance,
                                                  int slideRightDistance) {
        if (mineralLocation == MineralLocation.LEFT) {
            mecanum.SlideRight2(.5, slideLeftDistance, this);
        } else if (mineralLocation == MineralLocation.RIGHT) {
            mecanum.SlideLeft2(.5, slideRightDistance, this);
        }

        return AutonomousStates.BACK_AT_MIDDLE;
    }

    public AutonomousStates Mine() {
        watch.reset();
        // slide arm out
        robot.getSlide().setPower(-1);
        // run for 1000 milliseconds
        while ((watch.milliseconds() < 1000)  && opModeIsActive()) {
            idle();
        }

        robot.getSlide().setPower(0);

        robot.getTopFlip().setPosition(1);

        // drop the intake
        robot.getFrontFlip().setTargetPosition(FlatFlip + 700);
        robot.getFrontFlip().setPower(.7);

        // spin the intake to get minerals
        robot.getIntake().setPower(1);

        while (robot.getFrontFlip().isBusy() && opModeIsActive()) {
            idle();
        }

        watch.reset();
        robot.getSlide().setPower(-1);
        while ((watch.milliseconds() < 4000)  && opModeIsActive()) {
            idle();
            if (watch.milliseconds() > 500) {
                robot.getSlide().setPower(0);
            }

        }

        // make the robot twist left and right to help it dig
        mecanum.TurnLeft(.5, 300, this);
        mecanum.TurnRight(.5, 600, this);
        mecanum.TurnLeft(.5, 300, this);

        robot.getIntake().setPower(0);

        return AutonomousStates.MINED;

    }
}
