package com.edinaftcrobotics.navigation;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.edinaftcrobotics.drivetrain.TelemetryMounts;

import java.util.ArrayList;

public class PurePursuit extends Thread {

    private Mecanum mecanum;
    private ArrayList<double[]> points;
    private boolean on;
    private TelemetryMounts tm;
    //    This is in inches
    private double lookahead;
    private int oldl, olds, oldr;
    private String telemetry;

    public PurePursuit(Mecanum m, TelemetryMounts tm, double lookahead) {

        mecanum = m;
        this.tm = tm;
        this.lookahead = lookahead;

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
        mecanum.Stop();

    }

    public void shutDown() {

        cancel();
        on = false;

    }

    public String getTelemetry() {
        return telemetry;
    }

    public void run() {

        on = true;

        oldl = 0;
        oldr = 0;
        olds = 0;

//        Our lookahead is basically a circle, and the lookahead point will be the intersection
//        of the circle and the line that the robot is following. Since there are two intersections usually,
//        the point closest to the next segment is going to be the one chosen as lookahead point


//        tr is our target rotation or target heading
        double tr = 0;
        while (on) {
            String temp = "";
            int leftPos = mecanum.getBL().getCurrentPosition();
            int strafePos = mecanum.getFR().getCurrentPosition();
            int rightPos = mecanum.getFL().getCurrentPosition();

//        Taking the change in the encoder values
            int left = leftPos - oldl;
            int strafe = strafePos - olds;
            int right = rightPos - oldr;

//        Setting old positions to be used in the next iteration
            oldl = leftPos;
            olds = strafePos;
            oldr = rightPos;

            tm.update(-right, left, -strafe);

            double[] lookaheadPoint;
            if (points.size() > 1) {

                double x = points.get(1)[0] - tm.getX();
                double y = points.get(1)[1] - tm.getY();

                double toLineEnd = Math.sqrt(x * x + y * y);

                if (toLineEnd < lookahead) {
                    points.remove(0);
                }

                temp += "Distance to nearest point: " + toLineEnd + "\n";

                double xPositive;
                double yPositive;
                double xNegative;
                double yNegative;

                double r = lookahead;
                double h = tm.getX();
                double k = tm.getY();

                double m = 0;
                double a2 = 0;
                double b2 = 0;
                double c2 = 0;
                double b = 0;

//              Testing for vertical line
                boolean vertical = points.get(0)[0] == points.get(1)[0];
                boolean intersects;
                if (vertical) {
                    temp += "Current line is vertical\n";
                    double axis = points.get(0)[0];

                    xPositive = axis;
                    xNegative = axis;

                    yPositive = Math.sqrt((r + axis - h) * (r - axis + h)) + k;
                    yNegative = -Math.sqrt((r + axis - h) * (r - axis + h)) + k;

//                    Testing if the circle doesn't intersect;
                    intersects = !Double.isNaN(yPositive);

                } else {
                    temp += "Current line is not verticle\n";
                    m = (points.get(1)[1] - points.get(0)[1]) / (points.get(1)[0] - points.get(0)[0]);
                    b = -m * points.get(0)[0] + points.get(0)[1];

//                These variables are for the quadratic formula
                    a2 = 1 + m * m;
                    b2 = 2 * (m * b - h - m * k);
                    c2 = b * b + k * k + h * h - r * r - 2 * b * k;

//                This is solving the circle/line intersection with both positive and negative solutions
                    xPositive = (-b2 + Math.sqrt(b2 * b2 - 4 * a2 * c2)) / (2 * a2);

                    yPositive = m * xPositive + b;

                    xNegative = (-b2 - Math.sqrt(b2 * b2 - 4 * a2 * c2)) / (2 * a2);

                    yNegative = m * xNegative + b;

                    intersects = !Double.isNaN(Math.sqrt(b2 * b2 - 4 * a2 * c2));
                }

                double tx = points.get(1)[0];
                double ty = points.get(1)[1];

//                This tests if the circle doesn't intersect the line
                if (!intersects) {
                    temp += "Does not intersect\n";
                    if (vertical) {
                        lookaheadPoint = new double[]{
                                tx,
                                tm.getY()
                        };
                    } else {
//                    Just getting the closest point to the line

//                        Checking if perpendicular line is vertical
                        if (m == 0) {
                            lookaheadPoint = new double[]{
                                    tm.getX(),
                                    ty
                            };
                        } else {
//                            This is if the perpendicular line is not vertical
                            double pm = -1 / m;
                            double pb = -pm * h + k;

                            lookaheadPoint = new double[]{
                                    (pb - b) / (m - pm),
                                    m * (pb - b) / (m - pm) + b
                            };
                        }
                    }

                } else {
                    temp += "Circle intersects\n";
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
                if (dist(lookaheadPoint[0] - tm.getX(), lookaheadPoint[1] - tm.getY()) < 2) {
                    points.remove(0);
                }

            } else {
                lookaheadPoint = new double[]{
                        tm.getX(),
                        tm.getY()
                };
            }


            if (points.size() >= 1) {
                double angle = Math.atan2(lookaheadPoint[1] - tm.getY(), lookaheadPoint[0] - tm.getX());
                double dist = dist(lookaheadPoint[0] - tm.getX(), lookaheadPoint[1] - tm.getY());
                dist = dist > 1 ? 1 : dist < -1 ? -1 : dist;

//            Rotation
                double r = tm.getHeading();
//            Target Rotation
                tr = (lookaheadPoint.length == 3) ? points.get(1)[2] : tr;

                mecanum.assistedDrive(
                        Math.cos(angle) * dist,
                        -Math.sin(angle) * dist,
                        points.get(1).length == 3 ?
                                (loop(tr - r) > loop(r - tr) ?
                                        r - tr * 10
                                        :
                                        tr - r * 10
                                ) / 180
                                :
                                0,
                        tm.getHeading()
                );

                temp += "Nearest point: (" + points.get(1)[0] + ", " + points.get(1)[1] + ")\n";
                temp += "Lookahead point: (" + lookaheadPoint[0] + ", " + lookaheadPoint[1] + ")\n";
                temp += "Angle: " + angle + ", Lookahead: " + lookahead + "\n";
                temp += "X: " + tm.getX() + ", Y: " + tm.getY() + "\n";
                temp += "Power: " + dist + "\n";


            } else {
                temp += "Idle\n";
            }

            temp += "Path length: " + points.size() + "\n";

            telemetry = temp;

        }

    }

    private double dist(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    private double loop(double d) {

        return ((d % 360) + 360) % 360;

    }

}
