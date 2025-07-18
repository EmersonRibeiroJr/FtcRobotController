package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.localization.OdometryLocalizer;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Autonomous(name = "Odometry Auto")
public class AutonomousOpMode extends LinearOpMode {

    private MecanumDrive drive;
    private OdometryLocalizer odometry;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap);
        odometry = new OdometryLocalizer(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            odometry.update();

            double targetX = 24.0; // Exemplo: 24 polegadas pra frente
            double error = targetX - odometry.getX();

            double power = 0.3;
            if (Math.abs(error) > 0.5) {
                drive.drive(0, power * Math.signum(error), 0);
            } else {
                drive.stop();
            }

            telemetry.addData("X", odometry.getX());
            telemetry.addData("Y", odometry.getY());
            telemetry.addData("Heading", odometry.getHeadingDegrees());
            telemetry.update();
        }
    }
}
