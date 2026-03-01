package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlyWheelTuner extends OpMode {

    public DcMotorEx FlywheelMotor;

    public DcMotorEx FlywheelMotor2;

        public double highVelocity = 800;
        public double lowVelocity = 140;
        double TargetVelocity=highVelocity;
        double F=12.777; // RIGHT=12.777;
        double P=77.4; // RIGHT= 77.4
    PIDFCoefficients pidfCoefficients=new PIDFCoefficients(P,0,0,F);

    double[] stepSizes={10.0,1.0,0.1,0.001,0.0001};
        int stepIndex=1;
    @Override
        public void init() {
            FlywheelMotor=hardwareMap.get(DcMotorEx.class,"shooter2");
            FlywheelMotor2=hardwareMap.get(DcMotorEx.class,"shooter1");

        FlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            PIDFCoefficients pidfCoefficients=new PIDFCoefficients(P,0,0,F);
            FlywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
            FlywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FlywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
            FlywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        telemetry.addLine("Init Complete");
        }

        @Override
        public void loop() {
            if (gamepad1.yWasPressed()) {
                if (TargetVelocity == highVelocity) {
                    TargetVelocity = lowVelocity;
                } else {
                    TargetVelocity = highVelocity;
                }
            }

            if (gamepad1.bWasPressed()) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }

            if (gamepad1.dpadLeftWasPressed()) {
                F -= stepSizes[stepIndex];
            }

            if (gamepad1.dpadRightWasPressed()) {
                F += stepSizes[stepIndex];
            }

            if (gamepad1.dpadUpWasPressed()) {
                P += stepSizes[stepIndex];
            }
            if (gamepad1.dpadDownWasPressed()) {
                P -= stepSizes[stepIndex];
            }

            //Set the New PIDF Coefficients
            PIDFCoefficients pidfCoefficients=new PIDFCoefficients(P,0,0,F);
            FlywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
            FlywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
            //SET VELOICTY
            FlywheelMotor.setVelocity(TargetVelocity);

            double curVelocity=FlywheelMotor.getVelocity();
            double curVelocity2=FlywheelMotor2.getVelocity();
            double error=TargetVelocity-curVelocity;
            double error2=TargetVelocity-curVelocity2;

            telemetry.addData("Target Velocity", TargetVelocity);
            telemetry.addData("Current Velocity right", "%.2f", curVelocity);
            telemetry.addData("Current Velocity left", "%.2f", curVelocity2);
            telemetry.addData("Error right", "%.2f", error);
            telemetry.addData("Error left", "%.2f", error2);

            telemetry.addLine();
            telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
            telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
            telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);

        }
}


