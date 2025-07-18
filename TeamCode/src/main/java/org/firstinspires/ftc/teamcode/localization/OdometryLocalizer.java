package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OdometryLocalizer {
    private DcMotor leftPod, rightPod, rearPod;

    private double x = 0.0, y = 0.0, heading = 0.0;
    private int lastLeft = 0, lastRight = 0, lastRear = 0;

    static final double WHEEL_DIAMETER_INCHES = 1.89; // 48mm DeadWheel
    static final double COUNTS_PER_REV = 2000.0; // 2000 CPR quadratura
    static final double COUNTS_PER_INCH = COUNTS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

    static final double TRACK_WIDTH = 5.9;     // Calibrável
    static final double CENTER_WHEEL_OFFSET = 7; // Calibrável

    public OdometryLocalizer(HardwareMap hardwareMap) {
        leftPod = hardwareMap.get(DcMotor.class, "leftPod");
        rightPod = hardwareMap.get(DcMotor.class, "rightPod");
        rearPod = hardwareMap.get(DcMotor.class, "rearPod");

        resetEncoders();
    }

    public void resetEncoders() {
        leftPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        int currentLeft = leftPod.getCurrentPosition();
        int currentRight = rightPod.getCurrentPosition();
        int currentRear = rearPod.getCurrentPosition();

        int deltaLeft = currentLeft - lastLeft;
        int deltaRight = currentRight - lastRight;
        int deltaRear = currentRear - lastRear;

        lastLeft = currentLeft;
        lastRight = currentRight;
        lastRear = currentRear;

        double leftInches = deltaLeft / COUNTS_PER_INCH;
        double rightInches = deltaRight / COUNTS_PER_INCH;
        double rearInches = deltaRear / COUNTS_PER_INCH;

        double deltaHeading = (rightInches - leftInches) / TRACK_WIDTH;
        heading += deltaHeading;

        double forward = (rightInches + leftInches) / 2.0;
        double strafe = rearInches - (deltaHeading * CENTER_WHEEL_OFFSET);

        double sinH = Math.sin(heading);
        double cosH = Math.cos(heading);

        x += forward * cosH - strafe * sinH;
        y += forward * sinH + strafe * cosH;
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeadingRadians() { return heading; }
    public double getHeadingDegrees() { return Math.toDegrees(heading); }
}
