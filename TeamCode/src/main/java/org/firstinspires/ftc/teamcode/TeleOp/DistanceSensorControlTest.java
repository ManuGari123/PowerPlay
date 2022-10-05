package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Distance Sensor Arm", group = "ArmCodeTest")
public class DistanceSensorControlTest extends LinearOpMode {

    DcMotorEx ArmHeight;
    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor RBMotor;
    DcMotor LBMotor;
    Servo Turret;
    TouchSensor BottomReset;
    DistanceSensor Pole;

    double INCREMENT = 0.01;
    double max_pos_turret = 1.0;
    double min_pos_turret = 0;
    double home_turret = (max_pos_turret - min_pos_turret) / 2; //Starts exactly in between
    double turret_increment = INCREMENT;

    @Override
    public void runOpMode() throws InterruptedException {

        ArmHeight = hardwareMap.get(DcMotorEx.class, "ArmHeight");
        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        Turret = hardwareMap.servo.get("Turret");
        BottomReset = hardwareMap.get(TouchSensor.class, "BottomReset");
        Pole = hardwareMap.get(DistanceSensor.class, "Pole");
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        LFMotor.setDirection(DcMotor.Direction.REVERSE);
        ArmHeight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        while (opModeIsActive()) {
            //Code goes here:

            // Movement Base Gamepad1
            double powerMultiply;
            powerMultiply = 1 - gamepad1.right_trigger;

            // Movement Base Gamepad1
            double lateral = gamepad1.left_stick_x;
            double longitudinal = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double wheelPower = Math.hypot(lateral, longitudinal);
            double stickAngleRadians = Math.atan2(longitudinal, lateral);
            stickAngleRadians = stickAngleRadians - Math.PI / 4;
            double sinAngleRadians = Math.sin(stickAngleRadians);
            double cosAngleRadians = Math.cos(stickAngleRadians);
            double factor = 1 / Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians));
            double LFPower = (wheelPower * cosAngleRadians * factor + turn) * powerMultiply;
            LFMotor.setPower(LFPower);
            double RFPower = (wheelPower * sinAngleRadians * factor - turn) * powerMultiply;
            RFMotor.setPower(RFPower);
            double LBPower = (wheelPower * sinAngleRadians * factor + turn) * powerMultiply;
            LBMotor.setPower(LBPower);
            double RBPower = (wheelPower * cosAngleRadians * factor - turn) * powerMultiply;
            RBMotor.setPower(RBPower);

            //ARM CODE STARTS HERE:
            //Turret Left
            while (gamepad2.left_stick_x < 0) {
                home_turret += turret_increment;
                Turret.setPosition(home_turret);
            }

            //Turret Right
            while (gamepad2.left_stick_x > 0) {
                home_turret -= turret_increment;
                Turret.setPosition(home_turret);
            }

            //Top Pole Code
            double BottomDistance = Pole.getDistance(DistanceUnit.CM);
            if (gamepad2.a) {
                if (BottomDistance > 31) {
                    ArmHeight.setPower(-0.2);
                } else if (BottomDistance < 29) {
                    ArmHeight.setPower(0.2);
                } else if (BottomDistance <= 31 && BottomDistance >= 29){
                    //Add Dropper Code for Top Level Here Depending on Dropper Type(Servo, Motor, or Dual Control)
                }
            }

            //Middle Pole Code
            if (gamepad2.b) {
                if (BottomDistance > 21) {
                    ArmHeight.setPower(0.2);
                } else if (BottomDistance < 19) {
                    ArmHeight.setPower(-0.2);
                } else if (BottomDistance <= 31 && BottomDistance >= 29) {
                    //Add Dropper Code for Middle Level Here
                }
            }

            //Bottom Code Code
            if (gamepad2.a) {
                if (BottomDistance > 11) {
                    ArmHeight.setPower(0.2);
                } else if (BottomDistance < 9) {
                    ArmHeight.setPower(-0.2);
                } else if (BottomDistance <= 31 && BottomDistance >= 29){
                    //Add Dropper Code for Bottom Level Here
                }
            }



        }
    }
}