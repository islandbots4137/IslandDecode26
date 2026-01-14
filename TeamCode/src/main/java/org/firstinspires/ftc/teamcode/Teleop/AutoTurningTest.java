package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp// no usages

public class AutoTurningTest extends OpMode {

        GoBildaPinpointDriver odo;
        private double headingSetpoint = 0; // 3 usages
        DcMotor frontLeftMotor;
        DcMotor backLeftMotor;
        DcMotor frontRightMotor;
        DcMotor backRightMotor;


        @Override
        public void init() {
            frontLeftMotor  = hardwareMap.get(DcMotor.class, "leftFront");
            backLeftMotor   = hardwareMap.get(DcMotor.class, "leftBack");
            frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
            backRightMotor  = hardwareMap.get(DcMotor.class, "rightBack");
            frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            odo.setOffsets(0.2,-6.5, DistanceUnit.INCH);
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.REVERSED
            );
            odo.resetPosAndIMU();


        }

        @Override
        public void loop() {

            // Gets heading of robot
            odo.update();
            double headingDeg = odo.getHeading(AngleUnit.DEGREES);
            double headingRad = Math.toRadians(headingDeg);

            // Gets driver input
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            // Auto turning
            // Boolean to store if we want to auto turn
            boolean runAutoTurn;

// Gets target setpoint from input
            if (gamepad1.a)
            {
                headingSetpoint = 45;
                runAutoTurn = true;
            }
            else if (gamepad1.b) {
                headingSetpoint = 135;
                runAutoTurn = true;
            }
            else {
                runAutoTurn = false;
            }
            if (runAutoTurn) {

                double error = headingSetpoint - Math.toDegrees(headingRad);

                if (error > 180) {
                    error -= 360;
                } else if (error < -180) {
                    error += 360;
                }

                rx = 0.020 * error;
                rx = Math.min(Math.max(rx, -0.37), 0.37);
            }

// Field oriented mecanum drive controller
            double rotX = x * Math.cos(-headingRad) - y * Math.sin(-headingRad);
            double rotY = x * Math.sin(-headingRad) + y * Math.cos(-headingRad);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double leftFrontPower = (rotY + rotX - rx) / denominator;
            double leftRearPower = (rotY - rotX - rx) / denominator;
            double rightFrontPower = (rotY - rotX + rx) / denominator;
            double rightRearPower = (rotY + rotX + rx) / denominator;

            frontLeftMotor.setPower(leftFrontPower);
            backLeftMotor.setPower(leftRearPower);
            frontRightMotor.setPower(rightFrontPower);
            backRightMotor.setPower(rightRearPower);

        }
    }


