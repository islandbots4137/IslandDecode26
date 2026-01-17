package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelVelocityTuner extends OpMode {

    public DcMotorEx FlywheelMotor;
    public DcMotorEx FlywheelMotor2;

    // Presets you can save/toggle between
    public double highVelocity = 1350;
    public double lowVelocity  = 1250;

    // What we are currently commanding
    public double TargetVelocity = highVelocity;

    // Keep PIDF fixed (not tuned by buttons anymore)

    // Step sizes for velocity tuning
    private final double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    private int stepIndex = 1;

    @Override
    public void init() {
        FlywheelMotor  = hardwareMap.get(DcMotorEx.class, "shooter2");
        FlywheelMotor2 = hardwareMap.get(DcMotorEx.class, "shooter1");

        FlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FlywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FlywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {

        // Toggle between preset speeds
        if (gamepad1.yWasPressed()) {
            TargetVelocity = TargetVelocity+stepSizes.length;
        }
        if (gamepad1.xWasPressed()) {
            TargetVelocity = TargetVelocity-stepSizes.length;
        }
        // Optional: save the current target into presets
        if (gamepad1.bWasPressed()) {
            highVelocity = TargetVelocity;
        }
        if (gamepad1.aWasPressed()) {
            lowVelocity = TargetVelocity;
        }

        // Safety clamp so you don't accidentally go negative
        if (TargetVelocity < 0) TargetVelocity = 0;

        // Command BOTH motors
        FlywheelMotor.setVelocity(TargetVelocity);
        FlywheelMotor2.setVelocity(TargetVelocity);

        double curVelocity1 = FlywheelMotor.getVelocity();
        double curVelocity2 = FlywheelMotor2.getVelocity();

        telemetry.addData("Target Velocity", "%.3f", TargetVelocity);
        telemetry.addData("Current Velocity shooter2", "%.2f", curVelocity1);
        telemetry.addData("Current Velocity shooter1", "%.2f", curVelocity2);
        telemetry.addData("Error shooter2", "%.2f", (TargetVelocity - curVelocity1));
        telemetry.addData("Error shooter1", "%.2f", (TargetVelocity - curVelocity2));

    }
}
