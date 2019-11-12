package org.firstinspires.ftc.teamcode.teamcode2020;

/*
 * Created by 22tjiang on 9/13/19.
 */

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.game.robot.Convert;

import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.hardware.ColorSensor;


@TeleOp(name = "AutoRedBrick", group = "Auto") // RENAME //
//originally had it as TeleOp b/c Autonomous wasn't working, but changed back over
public class AutoRedBrick extends LinearOpMode {
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();
    private ColorSensor colorSensor;
    private TouchSensor touchSensor;

    public void runOpMode() throws InterruptedException {
        robot = new Robot2017();
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);
        robot.setTime(runtime);
        touchSensor = hardwareMap.touchSensor.get("touchSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        double distTravelled = 2.0;
        final double SCALE_FACTOR = 255;
        int stoneCount = 0;
        float hsvValues[] = {0F, 0F, 0F};
        double feederPow = 0;
        double distanceDown = 0;

        //*inputGameConfig();


        //Wait for the match to begin, presses start button
        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        while (opModeIsActive()) {
            Color.RGBToHSV((int)(colorSensor.red() * SCALE_FACTOR), (int) (colorSensor.green() * SCALE_FACTOR), (int) (colorSensor.blue() * SCALE_FACTOR), hsvValues);


            robot.composeIMUTelemetry();

            //robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGV(-1.3), robot.getHeading());
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(1), robot.getHeading());
            robot.gyrodrive.turn(0.7, -90);
            robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGV(-0.5), robot.getHeading());

            while (colorSensor.red() + colorSensor.blue() + colorSensor.green() > 525){
                robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(-.33), robot.getHeading());
                distanceDown += 0.33;
                stoneCount +=1;
            }

            robot.gyrodrive.vertical(-0.7, Convert.tileToYeetGV(.38), robot.getHeading());  // Back of robot is 0.33 tiles
            robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGV(-.8), robot.getHeading());  //TODO Find proper distance


            while (touchSensor.getValue() != 1) {
                robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(.5), robot.getHeading());
                feederPow = .5;
                robot.rfeedMotor.setPower(-feederPow);
                robot.lfeedMotor.setPower(feederPow);
            }

            feederPow = 0;

            robot.rfeedMotor.setPower(-feederPow);
            robot.lfeedMotor.setPower(feederPow);
            break;
        }
    }
}
