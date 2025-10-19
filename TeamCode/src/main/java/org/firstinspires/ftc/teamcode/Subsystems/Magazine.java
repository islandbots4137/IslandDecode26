package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Magazine {

    private final DcMotorEx motor;
    private int targetPosition = 0;

    public Magazine(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "magazine");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reset Encoder
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setTargetPosition(int targetPosition) {
        if (this.targetPosition != targetPosition) {
            this.targetPosition = targetPosition;
            motor.setTargetPosition(this.targetPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1.0);  // Edit power as desired
        }
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }
}
