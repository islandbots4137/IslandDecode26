package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class NewConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()



            .mass(9.97)
            // tune in order of PIDF. Translation,Heading, Drive, Centripetal.
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.01, 0.5))
            //.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0))
            // tune in order of PIDF. Translation,Heading, Drive, Centripetal.
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0.001, 0.03))
            //.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0))

            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.02, 0.002))
            //.secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0))
            //Uncomment one by one while tuning pid.
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.006,0.0005,0.001,0.6,0.0006))
            //.secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0,0.01,0.6,0.01))

            .centripetalScaling(0.0008) // RETUNE this


            //After tuning all of above run tests.
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .forwardZeroPowerAcceleration(-52.62)
            .lateralZeroPowerAcceleration(-86.47)
            .useSecondaryDrivePIDF(true);
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
            .xVelocity(71.75)
            .yVelocity(47.95);

    //

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    // USE PORTS 0 and 3 for encoders. Mechanical advantages of those.
//    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
//            .forwardEncoder_HardwareMapName("rightBack")
//            .strafeEncoder_HardwareMapName("leftBack")
//            .strafePodX(-6.5)
//            .forwardPodY(.2)
//            .forwardTicksToInches(0.0020087)
//            .strafeTicksToInches(0.0020468531)
//            .forwardEncoderDirection(Encoder.FORWARD)
//            .strafeEncoderDirection(Encoder.REVERSE)
//            .IMU_HardwareMapName("imu")
//            .IMU_Orientation(
//                    new RevHubOrientationOnRobot(
//                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
//                    )
//            );
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6.5)
            .strafePodX(0.2)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
