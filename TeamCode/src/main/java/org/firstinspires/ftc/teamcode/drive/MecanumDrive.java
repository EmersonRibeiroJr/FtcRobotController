package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {
    private DcMotor FL0, FR1, BL2, BR3;

    public MecanumDrive(HardwareMap hardwareMap) {
        FL0 = hardwareMap.get(DcMotor.class, "FL0");
        FR1 = hardwareMap.get(DcMotor.class, "FR1");
        BL2 = hardwareMap.get(DcMotor.class, "BL2");
        BR3 = hardwareMap.get(DcMotor.class, "BR3");

        FR1.setDirection(DcMotor.Direction.REVERSE);
        BR3.setDirection(DcMotor.Direction.REVERSE);
    }

    public void drive(double x, double y, double rotation) {
        double frontLeft = y + x + rotation;
        double frontRight = y - x - rotation;
        double backLeft = y - x + rotation;
        double backRight = y + x - rotation;

        double max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)),
                Math.max(Math.abs(backLeft), Math.abs(backRight)));

        if (max > 1.0) {
            frontLeft /= max;
            frontRight /= max;
            backLeft /= max;
            backRight /= max;
        }

        FL0.setPower(frontLeft);
        FR1.setPower(frontRight);
        BL2.setPower(backLeft);
        BR3.setPower(backRight);
    }

    public void stop() {
        FL0.setPower(0);
        FR1.setPower(0);
        BL2.setPower(0);
        BR3.setPower(0);
    }
}
