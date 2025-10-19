package org.firstinspires.ftc.teamcode.Testing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Magazine;
@TeleOp(name = "Magazine Tuner")
public class MagazineTest extends LinearOpMode {

    public static int targetPosition = 0;
    public static double power = 1.0;
   // public static int tolerance = 10;

    @Override
    public void runOpMode() {
        Magazine magazine = new Magazine(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            magazine.setTargetPosition(targetPosition);

            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", magazine.getCurrentPosition());
            telemetry.update();
        }
    }
}
