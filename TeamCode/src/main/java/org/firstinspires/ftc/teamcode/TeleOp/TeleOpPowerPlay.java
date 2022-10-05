package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Joey-TeleOp", group = "Competition")
public class TeleOpPowerPlay extends OpMode {
    org.firstinspires.ftc.teamcode.TeleOp.Joey Joey = new Joey();

    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor RBMotor;
    DcMotor LBMotor;

    Servo Turret;
    Servo ArmHeight;


    //If  turret is used with servo use these controls:
    double MAX_POS = 1.0;
    double MIN_POS = 0.0;
    double INCREMENT = 0.01;
    double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean Up = gamepad1.dpad_up;
    boolean Down = gamepad1.dpad_down;
    boolean Left = gamepad1.dpad_left;
    boolean Right = gamepad1.dpad_right;
    double max_pos_turret = 1.0;
    double min_pos_turret = 0;
    double home_turret = (max_pos_turret - min_pos_turret) / 2; //Starts exactly in between
    double turret_increment = INCREMENT;


    @Override
    public void init() {
        Joey.HARDWARE_INITIALIZE_BASE();
        Joey.HARDWARE_INITIALIZE_TURRET_ARMHEIGHT();
        Joey.RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Joey.LFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //      Turret = hardwareMap.get(Servo.class, "Turret");


    }


    @Override
    public void loop() {

        // Movement Base Gamepad1
        Joey.TELEOP_TRIGCONSTANTS();


        // Gamepad 2 Controls:
        //Arm Controls(Servo):

        //ArmHeight Up
        while (gamepad2.left_stick_y < 0) {

            position -= INCREMENT;
            ArmHeight.setPosition(position);

        }

        //ArmHeight Down
        while (gamepad2.left_stick_y > 0) {

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

        if (position <= MAX_POS) {
            position = MIN_POS;
            Down = Up;   // Switch ramp direction
        }
    }
    //Turret + Arm Controls(Motor)
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