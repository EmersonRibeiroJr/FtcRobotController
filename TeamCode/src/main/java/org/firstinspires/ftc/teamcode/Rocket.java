package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Rocket", group="Linear OpMode")
public class Rocket extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL0 = null;
    private DcMotor FR1 = null;
    private DcMotor BL2 = null;
    private DcMotor BR3 = null;

    public void runOpMode(){

        FL0 = hardwareMap.get(DcMotor.class, "FL0");
        FR1 = hardwareMap.get(DcMotor.class, "FR1");
        BL2 = hardwareMap.get(DcMotor.class, "BL2");
        BR3 = hardwareMap.get(DcMotor.class, "BR3");

        FL0.setDirection(DcMotorSimple.Direction.FORWARD);
        BL2.setDirection(DcMotorSimple.Direction.FORWARD);
        FR1.setDirection(DcMotorSimple.Direction.REVERSE);
        BR3.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Inicio");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            double max;


            double axial =  -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw =     gamepad1.right_stick_x;




            double leftFrontPower =  axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower =   axial - lateral + yaw;
            double rightBackPower =  axial + lateral - yaw;


            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0){
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }


            FL0.setPower(leftFrontPower);
            FR1.setPower(rightFrontPower);
            BL2.setPower(leftBackPower);
            BR3.setPower(rightBackPower);


            telemetry.addData("Status", "Run time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();




        }
    }
}
