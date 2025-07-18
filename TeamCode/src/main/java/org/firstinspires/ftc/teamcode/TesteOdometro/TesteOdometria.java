package org.firstinspires.ftc.teamcode.TesteOdometro;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "TesteOdometria", group = "Sensor")
public class TesteOdometria extends LinearOpMode {

    // Motores de leitura (pods)
    private DcMotor leftPod, rightPod, rearPod;

    // Constantes de hardware
    static final double WHEEL_DIAMETER_INCHES = 1.89; // 48mm DeadWheel
    static final double COUNTS_PER_REV = 8000.0; // 2000 CPR quadratura
    static final double COUNTS_PER_INCH = COUNTS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES); // ≈ 1348.4

    // Constantes de posição do robô (você precisa calibrar)
    static final double TRACK_WIDTH = 5.9;     // Distância entre left e right pod (polegadas)
    static final double CENTER_WHEEL_OFFSET = -7; // Offset lateral do rear pod em relação ao centro do robô

    // Posição do robô
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0; // radianos

    // Últimas posições dos encoders
    int lastLeft = 0;
    int lastRight = 0;
    int lastRear = 0;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        // Inicializa os pods
        leftPod = hardwareMap.get(DcMotor.class, "leftPod");
        rightPod = hardwareMap.get(DcMotor.class, "rightPod");
        rearPod = hardwareMap.get(DcMotor.class, "rearPod");

        resetEncoders();

        waitForStart();

        while (opModeIsActive()) {
            updatePosition();

            telemetry.addData("X (in)", String.format("%.2f", x));
            telemetry.addData("Y (in)", String.format("%.2f", y));
            telemetry.addData("Heading (deg)", String.format("%.2f", Math.toDegrees(heading)));
            telemetry.update();
        }
    }

    private void resetEncoders() {
        leftPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void updatePosition() {
        // Leitura atual dos encoders
        int currentLeft = leftPod.getCurrentPosition();
        int currentRight = rightPod.getCurrentPosition();
        int currentRear = rearPod.getCurrentPosition();

        // Delta (diferença) desde a última leitura
        int deltaLeft = currentLeft - lastLeft;
        int deltaRight = currentRight - lastRight;
        int deltaRear = currentRear - lastRear;

        // Atualiza última leitura
        lastLeft = currentLeft;
        lastRight = currentRight;
        lastRear = currentRear;

        // Converte contagens em deslocamento linear (polegadas)
        double leftInches = deltaLeft / COUNTS_PER_INCH;
        double rightInches = deltaRight / COUNTS_PER_INCH;
        double rearInches = deltaRear / COUNTS_PER_INCH;

        // Calcula delta angular (heading)
        double deltaHeading = (rightInches - leftInches) / TRACK_WIDTH;
        heading += deltaHeading;

        // Calcula movimento linear médio
        double forward = (rightInches + leftInches) / 2.0;
        double strafe = rearInches - (deltaHeading * CENTER_WHEEL_OFFSET);

        // Atualiza posição global (X, Y)
        double sinH = Math.sin(heading);
        double cosH = Math.cos(heading);

        x += forward * cosH - strafe * sinH;
        y += forward * sinH + strafe * cosH;
    }
}
