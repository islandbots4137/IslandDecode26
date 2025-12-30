package org.firstinspires.ftc.teamcode.Constants;


import com.pedropathing.math.MathFunctions;

public final class TeleOpConstants {
    //Shooter Constants
    public static final double shooterVelocity=1200;
    public static final double shooterVelocityFar=1375;
    public static boolean ShooterRunning = false;
    public static double shooterSpeed(double goalDist)
    {
        return MathFunctions.clamp(0,0,0);
    }
    public static double hoodAngle(double goalDist)
    {
        return MathFunctions.clamp(0,0,0);
    }

    //MAGAZINE CONSTANTS
    public static int positions=0;
    public static boolean spun = false;
    public static int rotationTimes = 0;
    // Color Sensor Constants
    public static boolean wasTan = false;
    public static double magazinePower= 0.35;

    // Intake Constants
    public static boolean intakeOn = false;
    public static boolean intakeReverse = false;
    public static boolean autoIntake = false;
    public static double intakeIn=-0.75;
    public static double intakeOut=0.75;


    // PUSHER
    public static boolean PusherScore = false;
    public static double servoForward=0;
    public static double servoBack=0.6;







}
