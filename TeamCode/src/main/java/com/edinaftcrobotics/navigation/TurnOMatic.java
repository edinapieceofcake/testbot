package com.edinaftcrobotics.navigation;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Timer;
import java.util.TimerTask;

public class TurnOMatic {
    BNO055IMU imu = null;
    Mecanum mecanum = null;
    Telemetry telemetry = null;
    private double error = 0;
    private double startAngle = 0;
    private double endAngle = 0;
    private double derivative = 0;
    private double integral = 0;
    private double previousError = 0;
    private double currentAngle = 0;
    private double timerLength = 200;
    private double Kp = 0.2, Ki = 0.01, Kd = 1; // PID constant multipliers
    private double output = 0;
    private double previousOutput = 0;
    private long previousTime;
    private double firstValue = 0;
    private boolean firstRun = true;
    private Timer timer = null;
    private TimerTask timerTask = null;
    private LinearOpMode opMode = null;
    private double motorRatio = .7;

    public TurnOMatic(BNO055IMU imu, Mecanum mecanum, Telemetry telemetry, double toWnatAngle, LinearOpMode opMode, double motorRatio) {
        this.imu = imu;
        this.mecanum = mecanum;
        this.telemetry = telemetry;
        startAngle = currentAngle = GetImuAngle();
        endAngle = toWnatAngle;
        this.opMode = opMode;
        this.motorRatio = motorRatio;
        this.mecanum.StopAndResetMotors2();

        SetupTimerTask();
        StartTimer();
        // gives time for the timer to fire at least once
        this.opMode.sleep(300);
    }

    public void Turn(double closeEnough, long ticksToWait) {
        double currentRatio = 1;
        int steadyState = 0;

        while ((steadyState <= 5) && opMode.opModeIsActive()){
        //while ((Math.abs(currentRatio) > closeEnough) && opMode.opModeIsActive()){
            currentRatio = (previousOutput - output) / firstValue;

            if ((Math.abs(currentRatio) <= closeEnough)) {
                steadyState++;
            } else {
                steadyState = 0;
            }
            telemetry.addData("S, C, E, A Angle", "%f, %f, %f, %f", startAngle, currentAngle, endAngle, GetImuAngle());
            telemetry.addData("PreviousOutput: ", previousOutput);
            telemetry.addData("Output: ", output);
            telemetry.addData("CurrentRatio", currentRatio);
            telemetry.addData("Proportional", error);
            telemetry.addData("Integral", integral);
            telemetry.addData("Derivative", derivative);
            telemetry.addData("Output Difference: ", previousOutput - output);
            double answer = Range.clip(currentRatio, -1.0, 1.0) * motorRatio;
            double left = answer;
            double right = -answer;
            telemetry.addData("Left Power: ", "%f", answer);
            telemetry.addData("Right Power: ", "%f", -answer);

            mecanum.Move(left , right);
            telemetry.update();
            opMode.sleep(200);
        }

        mecanum.Stop();

        StopTimer();
    }

    private double GetImuAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle;
    }

    private void SetupTimerTask() {
        timerTask = new TimerTask() {
            @Override
            public void run() {
                // we think the integral value is not being cleared
                // so we are adding a check to set it to zero
                if (firstRun) {
                    if (Double.isNaN(integral)){
                        integral = 0;
                    }
                }

                // basic pid code for getting the angle and computing the output
                // for motor speed
                currentAngle = GetImuAngle();

                error = endAngle - currentAngle;
                integral = integral + (error * 200);
                derivative = (error - previousError) / 200;

                previousOutput = output;
                output = (Kp * error) + (Ki * integral) + (Kd * derivative);

                if (firstRun) {
                    firstValue = Math.abs(previousOutput - output);
                    firstRun = false;
                }

                previousError = error;
            }
        };
    }

    private void StartTimer() {
        error = 0;
        derivative = 0;
        integral = 0;
        previousError = 0;
        currentAngle = 0;
        output = 0;
        previousOutput = 0;
        previousTime = System.currentTimeMillis();
        firstRun = true;
        firstValue = 0;

        timer = new Timer();
        timer.scheduleAtFixedRate(timerTask, 200, 200);
    }

    private void StopTimer() {

        timer.cancel();
    }
}
