package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TeleOp.Joey;

@Disabled
@Autonomous(name = "Encoders Auto - Original", group = "Encoders Autonomous")
public class AutonomousEncoders extends LinearOpMode {

    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor RBMotor;
    DcMotor LBMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        Joey Joey = new Joey();

        Joey.HARDWARE_INITIALIZE_BASE();

        Joey.ENCODERS_STARTUP();

//**************************************************************************************************************************************************************************

        waitForStart();


        Joey.FORWARD_ENCODERS_AUTONOMOUS(0.7, 20);

        sleep(1000);

        Joey.LEFT_ENCODERS_AUTONOMOUS(0.7, 20);

        sleep(1000);

        Joey.BACKWARDS_ENCODERS_AUTONOMOUS(0.7, 20);

        sleep(1000);

        Joey.RIGHT_ENCODERS_AUTONOMOUS(0.7, 20);

        sleep(1000);

        Joey.FORWARD_ENCODERS_AUTONOMOUS(0.7, 20);

    }
}