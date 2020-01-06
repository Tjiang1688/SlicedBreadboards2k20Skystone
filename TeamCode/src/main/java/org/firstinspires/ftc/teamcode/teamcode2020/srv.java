package org.firstinspires.ftc.teamcode.teamcode2020;

/**
 * Created by theCoolestKidEver on 9/13/19.
 */

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.game.robot.Convert;
import com.qualcomm.ftccommon.SoundPlayer;
import org.firstinspires.ftc.teamcode.game.robot.StartPosition;

import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "srv", group = "auto")
//originally had it as TeleOp b/c Autonomous wasn't working, but changed back over
public class srv extends LinearOpMode {
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();
    private ColorSensor colorSensor;
    private ColorSensor floorColorSensor;
    private TouchSensor touchSensor;
    private DistanceSensor distanceSensor;
    private Servo lServo;
    private Servo rServo;


    public void servoUp(){ lServo.setPosition(0.75f); rServo.setPosition(0.25f); lServo.setPosition(1.0f); rServo.setPosition(0.0f); }

    public void servoDown(){ lServo.setPosition(0.75f); rServo.setPosition(0.25f); lServo.setPosition(0.5f); rServo.setPosition(0.5f);}




    public void runOpMode() throws InterruptedException {
        robot = new Robot2017();
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);
        robot.setTime(runtime);

        //nameOfThingInCode = hardwareMap.typeOfThing.get("nameOfThingInConfiguration");
        touchSensor = hardwareMap.touchSensor.get("touchSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
        floorColorSensor = hardwareMap.get(ColorSensor.class, "floorColorSensor");
        lServo = hardwareMap.servo.get("lServo");
        rServo = hardwareMap.servo.get("rServo");


        double distTravelled = 2.0;
        double feederPow = 0;
        double feederWide = 0;
        ///////////////TODO before competition change floorBlue and close feeder
        int floorBlue = 600;
        final double SCALE_FACTOR = 255;
        float hsvValues[] = {0F, 0F, 0F};
        boolean skystone = false;
        int stoneCount = 0;
        double v1 = .4;


        //MAKE SURE FEEDER IS IN CLOSED POSITION BEFORE START

        //inputGameConfig();

        //Wait for the match to begin, press start button
        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        while (opModeIsActive()) {

            //color sensor scale factors
            Color.RGBToHSV((int)(colorSensor.red() * SCALE_FACTOR), (int) (colorSensor.green() * SCALE_FACTOR), (int) (colorSensor.blue() * SCALE_FACTOR), hsvValues);
            Color.RGBToHSV((int)(floorColorSensor.red() * SCALE_FACTOR), (int) (floorColorSensor.green() * SCALE_FACTOR), (int) (floorColorSensor.blue() * SCALE_FACTOR), hsvValues);

            robot.composeIMUTelemetry();


            servoUp();

            TimeUnit.MILLISECONDS.sleep(2000);


            servoDown();

            TimeUnit.MILLISECONDS.sleep(2000);


            servoUp();

            TimeUnit.MILLISECONDS.sleep(2000);


            lServo.setPosition(0.75f); rServo.setPosition(0.25f); lServo.setPosition(0.0f); rServo.setPosition(1.0f);


            break;
        }
    }
}