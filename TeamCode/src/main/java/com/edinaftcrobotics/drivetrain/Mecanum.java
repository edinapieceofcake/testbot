package com.edinaftcrobotics.drivetrain;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Mecanum {
    private DcMotor _frontLeft;
    private DcMotor _frontRight;
    private DcMotor _backLeft;
    private DcMotor _backRight;
    private Telemetry _telemetry;

    private double _currentPower = 1.0;

    //
    // This is our class that we use to drive the robot in autonomous and teleop.  It has a bunch
    // of helper method in it to help us.  The ones that end with the number 2 use the mode
    // run with encoders whilethe others use the run to position.  Except for the move, it just
    // moves the robot with some power.
    // The methods are:
    //
    //  SlideLeft2 - Slide left a certain distance using RUN_WITH_ENCODERS
    //  Move - moves the robot with a certain power
    //  SlideRight2 - slide right a certain distance using RUN_WITH_ENCODERS
    //  MoveForward - move forward a certain distance using RUN_TO_POSITION
    //  MoveForward2 - move forward a certain distance using RUN_WITH_ENCODERS
    //  MoveBackwards - move backwards a certain distance using RUN_TO_POSITION
    //  MoveBackwards2 - move backwards a certain distance using RUN_WITH_ENCODERS
    //  TurnRight - turn right with RUN_TO_POSITION
    //  TurnLeft - turn left with RUN_TO_POSITION
    //  Stop - stops the motors
    //
    public Mecanum(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, boolean isTeleop,
                   Telemetry telemetry)
    {
        _frontLeft = fl;
        _frontRight = fr;
        _backLeft = bl;
        _backRight = br;
        _telemetry = telemetry;

        if (isTeleop) {
            _backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            _frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void SlideLeft2(double power, int distance, LinearOpMode opMode) {
        // put the motors into run with encoders so they run with even power
        StopAndResetMotors2();

        int currentPosition =  Math.abs(_backRight.getCurrentPosition());
        int error = Math.abs((int)(distance * 0.95));
        Move(-power, power, power, -power);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while ((currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_backRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void Move(double left, double right){
        _frontLeft.setPower(left);
        _frontRight.setPower(right);
        _backLeft.setPower(left);
        _backRight.setPower(right);
    }

    public void Move(double fl, double fr, double bl, double br) {
        _frontLeft.setPower(fl);
        _frontRight.setPower(fr);
        _backLeft.setPower(bl);
        _backRight.setPower(br);
    }

    public void SlideRight2(double power, int distance, LinearOpMode opMode) {
        // put the motors into run with encoders so they run with even power
        StopAndResetMotors2();

        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        int error = Math.abs((int)(distance * 0.95));
        Move(power, -power, -power, power);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while ((currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void MoveForward(double power, int distance, LinearOpMode opMode) {
        // run with simple distance encoders as moving forward or backwards
        StopAndResetMotors();
        SetDistance(distance, distance, distance, distance);

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        Move(power, power, power, power);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while (_frontLeft.isBusy() && _frontRight.isBusy() && _backLeft.isBusy() && _backRight.isBusy() && (currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void MoveForward2(double power, int distance, LinearOpMode opMode) {
        // put the motors into run with encoders so they run with even power
        StopAndResetMotors2();

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        double currentPower = CalculateRampPower(power, distance, currentPosition);
        Move(currentPower, currentPower, currentPower, currentPower);

        while ((currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            currentPower = CalculateRampPower(power, distance, currentPosition);
            Move(currentPower, currentPower, currentPower, currentPower);
            opMode.idle();
        }

        Stop();
    }

    public void MoveBackwards(double power, int distance, LinearOpMode opMode) {
        // run with simple distance encoders as moving forward or backwards
        StopAndResetMotors();
        SetDistance(-distance, -distance, -distance, -distance);

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        Move(power, power, power, power);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while (_frontLeft.isBusy() && _frontRight.isBusy() && _backLeft.isBusy() && _backRight.isBusy() && (currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void MoveBackwards2(double power, int distance, LinearOpMode opMode) {
        // put the motors into run with encoders so they run with even power
        StopAndResetMotors2();

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());

        // ramp up and down the motor speed based on current position
        double currentPower = CalculateRampPower(power, distance, currentPosition);

        Move(-currentPower, -currentPower, -currentPower, -currentPower);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while ((currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            currentPower = CalculateRampPower(power, distance, currentPosition);
            Move(-currentPower, -currentPower, -currentPower, -currentPower);
            opMode.idle();
        }

        Stop();
    }

    public void TurnRight(double power, int distance, LinearOpMode opMode) {
        // run with simple distance encoders as moving forward or backwards
        StopAndResetMotors();
        SetDistance(distance, distance, -distance, -distance);

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        Move(power, power, power, power);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while (_frontLeft.isBusy() && _frontRight.isBusy() && _backLeft.isBusy() && _backRight.isBusy() && (currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void TurnLeft(double power, int distance, LinearOpMode opMode) {
        // run with simple distance encoders as moving forward or backwards
        StopAndResetMotors();
        SetDistance(-distance, -distance, distance, distance);

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        Move(power, power, power, power);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while (_frontLeft.isBusy() && _frontRight.isBusy() && _backLeft.isBusy() && _backRight.isBusy() && (currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void Stop() {
        _frontLeft.setPower(0);
        _frontRight.setPower(0);
        _backLeft.setPower(0);
        _backRight.setPower(0);
    }

    //
    // This method will calculate a power based on the current position and our maximum distance.
    // This a very simple motion profile method that uses distance to figure out different power
    // levels. More complex systems use derivitaves and time.
    // We want to ramp up the speed to a flat maxPower and then ramp down to zero.  We did this
    // to prevent the robot from losing traction and jerking.  We only use this when the robot motors are in
    // the run with encoders mode as the internal PID is being used for that instead of helping us
    // with distance.  So this is our real simple PID.  To learn more about what PID is, visit
    // https://en.wikipedia.org/wiki/PID_controller and to learn more about motion profiling visit
    //
    //
    private double CalculateRampPower(double maxPower, int distance, double currentDistance) {
        // out cutoffs on distance for this step up are:
        //  0-10% - .6 of power
        // 10-20% - .85 of power
        // 20-80% - full power
        // 80-90% - .85 power
        // 90-100% - .6 power
        if (currentDistance <= (distance * .10)) {
            return .6 * maxPower;
        } else if (currentDistance <= (distance * .20)) {
            return  .85 * maxPower;
        } else if (currentDistance <= (distance * .80)) {
            return maxPower;
        } else if (currentDistance <= (distance * .90)) {
            return .85 * maxPower;
        } else {
            return .6 * maxPower;
        }
    }

    //
    // This is our simple drive method that allows us to drive the robot in teleop
    //
    public void Drive(double leftStickX, double leftStickY, double rightStickY) {
        final double x = Math.pow(-leftStickX, 3.0);
        final double y = Math.pow(leftStickY, 3.0);

        final double rotation = Math.pow(-rightStickY, 3.0);
        final double direction = Math.atan2(x, y);
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        final double fl = speed * Math.sin(direction + Math.PI / 4.0) + rotation;
        final double fr = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
        final double bl = speed * Math.cos(direction + Math.PI / 4.0) + rotation;
        final double br = speed * Math.sin(direction + Math.PI / 4.0) - rotation;

        _frontLeft.setPower(-fl * _currentPower);
        _frontRight.setPower(-fr * _currentPower);
        _backLeft.setPower(-bl * _currentPower);
        _backRight.setPower(-br * _currentPower);
    }

    public void SetCurrentPower(double power){
        _currentPower = power;
    }

    //
    // These are our helper method to set the motors to what we need for the other steps
    // They are the three different ways you can run a motor
    //
    public void StopAndResetMotors() {
        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void StopAndResetMotors2() {
        _frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void StopAndResetMotors3() {
        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //
    // This method helps us set the distance on all the motors for things like turning and
    // moving forward.
    //
    private void SetDistance(int lf, int lb, int rf, int rb) {
        _frontLeft.setTargetPosition(lf);
        _frontRight.setTargetPosition(rf);
        _backLeft.setTargetPosition(lb);
        _backRight.setTargetPosition(rb);
    }
}
