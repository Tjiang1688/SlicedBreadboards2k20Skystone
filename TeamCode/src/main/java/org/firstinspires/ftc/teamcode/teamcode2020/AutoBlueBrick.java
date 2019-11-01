package org.firstinspires.ftc.teamcode.teamcode2020;

/**
 * Created by 22tjiang on 9/13/19.
 */

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


@TeleOp(name = "AutoBlueBrick", group = "auto")
//originally had it as TeleOp b/c Autonomous wasn't working, but changed back over
public class AutoBlueBrick extends LinearOpMode {
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();
    private ColorSensor colorSensor;
    private TouchSensor touchSensor;



    public void runOpMode() throws InterruptedException {
        robot = new Robot2017();
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);
        robot.setTime(runtime);

        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        touchSensor = hardwareMap.touchSensor.get("touchSensor");


        double distTravelled = 2.0;
        double feederPow = 0;

        //inputGameConfig();


        //Wait for the match to begin, presses start button
        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        while (opModeIsActive()) {


            robot.composeIMUTelemetry();
            /*

            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(2), robot.getHeading());
            telemetry.log().add(String.valueOf(robot.getHeading()));
            robot.gyrodrive.turn(0.7, -90);
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(2), robot.getHeading());

             */
            
            if (colorSensor.argb() < 100){
                feederPow = .5;
            }
            while (touchSensor.getValue() == 0) {
                robot.rfeedMotor.setPower(feederPow);
                robot.lfeedMotor.setPower(-feederPow);
            }

            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(-2), robot.getHeading());




            //robot.gyrodrive.turn(0.7, -180);
            break;

        }




    }
}