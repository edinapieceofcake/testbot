package com.edinaftcrobotics.navigation;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.edinaftcrobotics.drivetrain.TelemetryMounts;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;

public class PurePursuit extends Thread {

    private Mecanum mecanum;
    private ArrayList<double[]> points;
    private boolean on;
    private TelemetryMounts tm;
    //    This is in inches
    private final double LOOKAHEAD = 10;

    public PurePursuit(Mecanum m, TelemetryMounts tm) {

        mecanum = m;
        this.tm = tm;

    }

    public boolean isBusy() {

        return points.size() > 0;

    }

    public ArrayList<double[]> getPoints() {

        return points;

    }

    public void moveTo(ArrayList<double[]> points) {

        this.points = points;

    }

    public void cancel() {

        points = new ArrayList<>();
        mecanum.Move(0, 0, 0, 0);

    }

    public void shutDown() {

        cancel();
        on = false;

    }

    public void run() {

        on = true;


//        Our lookahead is basically a circle, and the lookahead point will be the intersection
//        of the circle and the line that the robot is following. Since there are two intersections usually,
//        the point closest to the next segment is going to be the one chosen as lookahead point


//        tr is our target rotation or target heading
        double tr = 0;
        while (on) {

            double[] lookaheadPoint;
            if (points.size() > 1) {

                double x = points.get(1)[0] - tm.getX();
                double y = points.get(1)[1] - tm.getY();

                double toLineEnd = Math.sqrt(x * x + y * y);

                if (toLineEnd > LOOKAHEAD) {
                    points.remove(0);
                }

                double m = (points.get(1)[1] - points.get(0)[1]) / (points.get(1)[0] - points.get(0)[0]);
                double b = -m * points.get(0)[0] + points.get(0)[1];

                double r = LOOKAHEAD;
                double h = tm.getX();
                double k = tm.getY();

//                These variables are for the quadratic formula
                double a2 = 1 + m * m;
                double b2 = 2 * (m * b - h - m * k);
                double c2 = b * b + k * k + h * h - r * r - 2 * b * k;

//                This is solving the circle/line intersection with both positive and negative solutions
                double xPositive = (-b2 + Math.sqrt(b2 * b2 - 4 * a2 * c2)) / (2 * a2);

                double yPositive = m * xPositive + b;

                double xNegative = (-b2 - Math.sqrt(b2 * b2 - 4 * a2 * c2)) / (2 * a2);

                double yNegative = m * xNegative + b;
                double tx = points.get(1)[0];
                double ty = points.get(1)[1];


//                This tests if the circle intersects the line at all
                if (Math.sqrt(b2 * b2 - 4 * a2 * c2) == Double.NaN) {

//                    Just getting the closest point to the line
                    double pm = -1 / m;
                    double pb = -pm * h + k;

                    lookaheadPoint = new double[]{
                            (pb - b) / (m - pm),
                            m * (pb - b) / (m - pm) + b
                    };

                } else {
//                Compares the two intersection points
                    if (dist(tx - xPositive, ty - yPositive) < dist(tx - xNegative, ty - yNegative)) {
                        lookaheadPoint = new double[]{
                                xPositive,
                                yPositive
                        };
                    } else {
                        lookaheadPoint = new double[]{
                                xNegative,
                                yNegative
                        };
                    }
                }


            } else if (points.size() == 1) {

                lookaheadPoint = points.get(0);

            } else {

                lookaheadPoint = new double[]{
                        tm.getX(),
                        tm.getY()
                };
            }

            double angle = Math.atan2(lookaheadPoint[1] - tm.getY(), lookaheadPoint[0] - tm.getX());
            double dist = dist(lookaheadPoint[0] - tm.getX(), lookaheadPoint[1] - tm.getY());

//            Rotation
            double r = tm.getHeading();
//            Target Rotation
            tr = (lookaheadPoint.length == 3) ? points.get(1)[2] : tr;

            mecanum.assistedDrive(
                    Math.cos(angle) * dist,
                    Math.sin(angle) * dist,
                    points.get(1).length == 3 ?
                            (loop(tr - r) > loop(r - tr) ?
                                    r - tr
                                    :
                                    tr - r
                            ) / 180
                            :
                            0,
                    tm.getHeading()
            );

        }

    }

    public double dist(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    public double loop(double d) {

        return ((d % 360) + 360) % 360;

    }


}
