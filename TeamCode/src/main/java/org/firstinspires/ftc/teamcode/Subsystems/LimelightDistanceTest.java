package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@TeleOp

public class LimelightDistanceTest extends OpMode {

    private Limelight3A limelight3A;
    private double distance;
    private GoBildaPinpointDriver odo;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(8);
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
    public void start() {
        limelight3A.start();
    }
    @Override
    public void loop() {

        // get yaw from control hub IMU
        odo.update();
        double yawDeg = odo.getHeading(AngleUnit.DEGREES);
        limelight3A.updateRobotOrientation(yawDeg);
        // get latest limelight result, pipeline 8 for April tag 0
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botpose = llResult.getBotpose_MT2();

            telemetry.addData("Yaw (deg) [Pinpoint]", yawDeg);
            telemetry.addData("Pinpoint Status", odo.getDeviceStatus());
            telemetry.addData("Calculated Distance", distance);
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("Botpose", botpose.toString());
            List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
            telemetry.addData("Fiducials detected", fiducials.size());

            for (LLResultTypes.FiducialResult f : fiducials) {
                telemetry.addData(
                        "Tag ID " + f.getFiducialId(),
                        String.format("tx=%.1f°, ty=%.1f°, area=%.1f%%",
                                f.getTargetXDegrees(),
                                f.getTargetYDegrees(),
                                f.getTargetArea())
                );
            }
        }
        else {
            telemetry.addData("Yaw (deg) [Pinpoint]", yawDeg);
            telemetry.addData("Pinpoint Status", odo.getDeviceStatus());
            telemetry.addData("Limelight", "No valid result");
        }
        telemetry.update();
    }
    // (more code is below this in your file, but it's cut off in the screenshot)
}
