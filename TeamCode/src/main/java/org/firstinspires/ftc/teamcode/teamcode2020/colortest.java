package org.firstinspires.ftc.teamcode.teamcode2020;

/**
 * Created by 22tjiang on 9/13/19.
 */

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.game.robot.Convert;
import org.firstinspires.ftc.teamcode.game.robot.StartPosition;

import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "colortest", group = "auto")
//originally had it as TeleOp b/c Autonomous wasn't working, but changed back over
public class colortest extends LinearOpMode {
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();
    private ColorSensor colorSensor;
    private ColorSensor rightColorSensor;
    private TouchSensor touchSensor;
    private DistanceSensor distanceSensor;
    private DistanceSensor rightDistanceSensor;
    private DistanceSensor actualDistanceSensor;


    public void runOpMode() throws InterruptedException {
        robot = new Robot2017();
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);
        robot.setTime(runtime);
        touchSensor = hardwareMap.touchSensor.get("touchSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
        rightColorSensor = hardwareMap.get(ColorSensor.class, "colorSensorRight");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensorRight");
        actualDistanceSensor = hardwareMap.get(DistanceSensor.class, "actualDistance");

        double distTravelled = 2.0;
        double feederPow = 0;
        final double SCALE_FACTOR = 255;
        float hsvValues[] = {0F, 0F, 0F};
        boolean skystone = false;
        int stoneCount = 0;

        //inputGameConfig();


        //Wait for the match to begin, presses start button
        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        while (opModeIsActive()) {

            Color.RGBToHSV((int)(colorSensor.red() * SCALE_FACTOR), (int) (colorSensor.green() * SCALE_FACTOR), (int) (colorSensor.blue() * SCALE_FACTOR), hsvValues);
            Color.RGBToHSV((int)(rightColorSensor.red() * SCALE_FACTOR), (int) (rightColorSensor.green() * SCALE_FACTOR), (int) (rightColorSensor.blue() * SCALE_FACTOR), hsvValues);


            robot.composeIMUTelemetry();

            telemetry.addData("left red:", colorSensor.red());
            telemetry.addData("left green:", colorSensor.green());
            telemetry.addData("left blue:", colorSensor.blue());

            telemetry.addData("right red:", rightColorSensor.red());
            telemetry.addData("right green:", rightColorSensor.green());
            telemetry.addData("right blue:", rightColorSensor.blue());

            telemetry.addData("left dist", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("right dist", rightDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("right dist", rightDistanceSensor.getDistance(DistanceUnit.INCH));



            telemetry.update();


        }




    }
}