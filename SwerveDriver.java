package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class SwerveDriver extends OpMode
{
    Servo redServo;
	Servo blueServo;
	Servo greenServo;
	Servo yellowServo;
    Servo[] servoList = new Servo[]{redServo, blueServo, greenServo, yellowServo};

    DcMotor redMotor;
    DcMotor blueMotor;
    DcMotor greenMotor;
    DcMotor yellowMotor;
    DcMotor[] motorList = new DcMotor[]{redMotor, blueMotor, greenMotor, yellowMotor};

    public void init() {
        redServo = hardwareMap.get(ServoImplEx.class, "redServo");
        blueServo = hardwareMap.get(ServoImplEx.class, "blueServo");
        greenServo = hardwareMap.get(ServoImplEx.class, "greenServo");
        yellowServo = hardwareMap.get(ServoImplEx.class, "yellowServo");
    }
	
	public void move(float strafeX, float strafeY, float rotate) {
		float theta1 = (float) Math.atan2(strafeY + rotate, strafeX + rotate);
        float theta2 = (float) Math.atan2(strafeY + rotate, strafeX - rotate);
		float strafePower = (float) Math.sqrt(Math.pow(strafeX, 2)+Math.pow(strafeY, 2));

	}

    private float thetatoEncoder(float theta){

    }
}