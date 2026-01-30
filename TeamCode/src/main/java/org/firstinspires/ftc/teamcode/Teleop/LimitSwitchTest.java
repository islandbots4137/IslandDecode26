package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import static org.firstinspires.ftc.teamcode.Constants.TeleOpConstants.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "Limit Switch Test", group = "Test")
public class LimitSwitchTest extends LinearOpMode {

    private DigitalChannel limitSwitch;

    @Override
    public void runOpMode() {
        limitSwitch = hardwareMap.get(DigitalChannel.class, "magazineLimitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Status", "Initialized - Press Play");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean rawState = limitSwitch.getState();
            boolean magnetDetected = !rawState;  // Usually inverted

            telemetry.addData("Raw getState()", rawState);
            telemetry.addData("Magnet Detected", magnetDetected);
            telemetry.addLine();
            telemetry.addLine(magnetDetected ? ">>> MAGNET DETECTED <<<" : "No magnet");
            telemetry.update();
        }
    }
}