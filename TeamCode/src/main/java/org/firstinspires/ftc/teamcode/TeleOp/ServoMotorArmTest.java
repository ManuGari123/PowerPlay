package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo + Motor Arm(Servo Current)", group = "ArmCodeTest")
public class ServoMotorArmTest extends OpMode {

    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor RBMotor;
    DcMotor LBMotor;
    Servo Turret;
    Servo ArmHeight;


    //If  turret is used with servo use these controls:
    //SERVOS DO NOT HAVE SPEED CONTROLS
    //FOLLOWING CODE IS FOR BOTH TURRET AND BASE HAVING SERVOS.
    //RANGES MUST BE UPDATED AFTER TEST
    double MAX_POS = 1.0;
    double MIN_POS = 0.0;
    double INCREMENT = 0.01;
    double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    double max_pos_turret = 1.0;
    double min_pos_turret = 0;
    double home_turret = (max_pos_turret - min_pos_turret) / 2; //Starts exactly in between
    double turret_increment = INCREMENT;


    @Override
    public void init() {

        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        Turret = hardwareMap.servo.get("Turret");
        ArmHeight = hardwareMap.servo.get("ArmHeight");
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        LFMotor.setDirection(DcMotor.Direction.REVERSE);
        //      Turret = hardwareMap.get(Servo.class, "Turret");

    }


    @Override
    public void loop() {

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
        boolean Up = gamepad2.left_stick_y > 0;
        boolean Down = gamepad2.left_stick_y < 0;


        // Gamepad 2 Controls:
        //Arm Controls(Servo):

        //ArmHeight Down

        while (gamepad2.left_stick_y > 0) {

            position -= INCREMENT;
            ArmHeight.setPosition(position);

        }

        //ArmHeight Up
        while (gamepad2.left_stick_y < 0) {
            position += INCREMENT;
            ArmHeight.setPosition(position);
        }

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

        //Controls to prevent
        if (position >= MAX_POS) {
            position = MAX_POS;
            Up = Down;   // Switch ramp direction
        }

        if (position <= MIN_POS) {
            position = MIN_POS;
            Down = Up;   // Switch ramp direction


        }
    }
    //Motor Controls:
                /* if (Joey.Right) {
                                Joey.Turret.setPower(0.1);
                            } else if (Joey.Left) {
                                Joey.Turret.setPower(-0.1);
                            } else {
                                Joey.Turret.setPower(0);
                            }


                            //Arm up and Down Controls for Motor:

                            if (Joey.Up) {
                                Joey.ArmHeight.setPower(0.3);
                            } else if (Joey.Down) {
                                Joey.ArmHeight.setPower(-0.3);
                            } else {
                                Joey.ArmHeight.setPower(0);
                            }

                            */
}