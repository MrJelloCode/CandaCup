package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.TeleOpConstants;


public class IntakeSubsystem {
    private DcMotorEx intake;
    boolean servoState = false;
    boolean lastX = false;
    private Servo gate;

    private Gamepad gamepad;


    public IntakeSubsystem(HardwareMap hw, Gamepad gamepad) {
        intake = hw.get(DcMotorEx.class, TeleOpConstants.Intake.INTAKE_MOTOR_NAME);
        gate = hw.get(Servo.class, TeleOpConstants.Intake.GATE_SERVO_NAME);

        this.gamepad = gamepad;
    }
    public void openGate(){
        gate.setPosition(TeleOpConstants.Intake.GATE_OPEN);
    }

    public void closeGate(){
        gate.setPosition(TeleOpConstants.Intake.GATE_CLOSE);
    }

    public void stop(){
        intake.setPower(0);
    }


    public void teleUpdate() {
        if (gamepad.right_trigger > 0.05) {
            intake.setPower(TeleOpConstants.Intake.POWER);          // intake in, proportional
        } else if (gamepad.left_trigger > 0.05) {
            intake.setPower(-TeleOpConstants.Intake.POWER);         // reverse out, proportional
        } else {
            intake.setPower(0);           // off
        }


        if (gamepad.x && !lastX) {
            servoState = !servoState;
        }
        lastX = gamepad.x;

        // Apply correct position
        if (servoState) {
            gate.setPosition(TeleOpConstants.Intake.GATE_CLOSE);
        } else {
            gate.setPosition(TeleOpConstants.Intake.GATE_OPEN);
        }    }

    public void autoPower(double power){
        intake.setPower(-power);
    }
}