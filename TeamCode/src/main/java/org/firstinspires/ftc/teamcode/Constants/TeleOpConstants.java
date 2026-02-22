package org.firstinspires.ftc.teamcode.Constants;



import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public final class TeleOpConstants {
    //Shooter Constants
    public static final double shooterVelocity=1190;
    public static final double shooterVelocityFar=1300;
    public static boolean ShooterRunning = false;
    public static boolean ShooterRunningFast = false;

    //MAGAZINE CONSTANTS

    public static int positions=0;
    public static boolean spun = false;
    public static int rotationTimes = 0;
    // Color Sensor Constants
    public static boolean wasTan = false;
    public static double magazinePower= -0.3;

    // Intake Constants
    public static boolean intakeOn = false;
    public static boolean intakeReverse = false;
    public static boolean autoIntake = false;
    public static double intakeIn=-0.80;
    public static double intakeOut=0.5;

    public static double INDEX_POWER = 0.20;
    public static double INDEX_TIMEOUT_S = 1.5;
    public static double headingSetpoint = 0;
    // Auto Turning Constants
    public static double TAG_TURN_kP = 0.03;   // start here
    public static double MAX_RX = 0.4;
    public static double TX_DEADBAND = 1.0;    // degrees
    public static double GREEN_POS = 0.5;     // aligned
    public static double NOT_READY_POS = 1; // not aligned
    public static double NO_TAG_POS = 0.10;    // no tag
    public static double ALIGN_THRESH_DEG = 5.0;

    // PUSHER
    public static boolean PusherScore = false;
    public static double servoForward=0.0;
    public static double servoPush=0;
    public static double servoLeave=0.55;
    public static double servoOpen=0;
    public static double servoClose=0.35;


    public static double servoBack=0.55;







}
