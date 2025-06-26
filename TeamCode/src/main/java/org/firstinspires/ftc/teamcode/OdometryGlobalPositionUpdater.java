package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class OdometryGlobalPositionUpdater implements Runnable {

    DcMotor verticalLeft, verticalRight, horizontal;

    static final double OFFSET_HORIZONTAL = 7.0; // distância lateral entre o pod horizontal e o centro do robô (em polegadas)
    static final double COUNTS_PER_RADIAN = 3600.0; // valor empírico baseado na distância entre pods verticais


    double countsPerInch;
    volatile double robotGlobalX, robotGlobalY, robotOrientationRadians;

    int previousVerticalLeft, previousVerticalRight, previousHorizontal;

    public OdometryGlobalPositionUpdater(DcMotor vl, DcMotor vr, DcMotor h, double cpi) {
        verticalLeft = vl;
        verticalRight = vr;
        horizontal = h;
        countsPerInch = cpi;

        previousVerticalLeft  = verticalLeft.getCurrentPosition();
        previousVerticalRight = verticalRight.getCurrentPosition();
        previousHorizontal    = horizontal.getCurrentPosition();
    }

    @Override
    public void run() {
        while (!Thread.interrupted()) {

            int currentVL = verticalLeft.getCurrentPosition();
            int currentVR = verticalRight.getCurrentPosition();
            int currentH  = horizontal.getCurrentPosition();

            int deltaVL = currentVL - previousVerticalLeft;
            int deltaVR = currentVR - previousVerticalRight;
            int deltaH  = currentH  - previousHorizontal;

            previousVerticalLeft  = currentVL;
            previousVerticalRight = currentVR;
            previousHorizontal    = currentH;

            double deltaAngle = (deltaVR - deltaVL) / 2.0 / COUNTS_PER_RADIAN;
            robotOrientationRadians += deltaAngle;

            double forward = (deltaVL + deltaVR) / 2.0;
            double strafe = deltaH - (OFFSET_HORIZONTAL * deltaAngle);

            double dx = forward * Math.cos(robotOrientationRadians) - strafe * Math.sin(robotOrientationRadians);
            double dy = forward * Math.sin(robotOrientationRadians) + strafe * Math.cos(robotOrientationRadians);

            robotGlobalX += dx / countsPerInch;
            robotGlobalY += dy / countsPerInch;

            try {
                Thread.sleep(10); // 100 updates por segundo
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public double getX() { return robotGlobalX; }
    public double getY() { return robotGlobalY; }
    public double getOrientation() { return robotOrientationRadians; }
}

