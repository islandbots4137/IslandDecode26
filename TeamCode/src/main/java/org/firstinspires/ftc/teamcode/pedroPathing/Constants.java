package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
         .mass(10.61)
         // tune in order of PIDF. Translation,Heading, Drive, Centripetal.
            .translationalPIDFCoefficients(new PIDFCoefficients(0.005, 0, 0.01, 0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0))

            .headingPIDFCoefficients(new PIDFCoefficients(0.5, 0.01, 0.1, 0.1))
         //   .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0))
            //Uncomment one by one while tuning pid.
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.2,0.0,0.02,0.6,0.0))
         //   .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0,0.01,0.6,0.01))

            .centripetalScaling(0.000065)

            //After tuning all of above run tests.
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .forwardZeroPowerAcceleration(-38.44164)
            .lateralZeroPowerAcceleration(-78.98164)
        .   useSecondaryDrivePIDF(true);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            // REMEMBER TO REVERSE ONCE WE TEST ON THE ROBOT
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(88.3628778)
            .yVelocity(69.0127749);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    // USE PORTS 0 and 3 for encoders. Mechanical advantages of those.
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("leftBack")
            .strafeEncoder_HardwareMapName("rightBack")
            .strafePodX(-5.2)
            .forwardPodY(1.2)
            .forwardTicksToInches(0.001984)
            .strafeTicksToInches(-0.001987)
           .forwardEncoderDirection(Encoder.FORWARD)

            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            );
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
