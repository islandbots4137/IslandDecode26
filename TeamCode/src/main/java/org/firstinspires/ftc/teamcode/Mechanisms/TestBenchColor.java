package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestBenchColor {
    NormalizedColorSensor colorSensor;
    public enum DetectedColor
    {
        RED,
        Blue,
        YELLOW,
        TAN,
        UNKNOWN

    }
    public void init(HardwareMap hwMap)
    {
        colorSensor=hwMap.get(NormalizedColorSensor.class,"colorSensor");
        colorSensor.setGain(3);
    }
    public DetectedColor getDetectedColor(Telemetry telemetry)
    {
        NormalizedRGBA colors=colorSensor.getNormalizedColors(); // 4 values
        float normRed, normGreen, normBlue;
        normRed=colors.red/colors.alpha;
        normGreen=colors.green/colors.alpha;
        normBlue=colors.blue/colors.alpha;
        if (normRed > 0.35 && normGreen > 0.25 && normBlue < 0.20)
        {
            return DetectedColor.TAN;
        }
        telemetry.addData("red",normRed);
        telemetry.addData("green",normGreen);
        telemetry.addData("blue",normBlue);
        return DetectedColor.UNKNOWN;
    }
}
