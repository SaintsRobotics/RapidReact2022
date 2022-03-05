package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants.OperatorButtonConstants;;


/**
 * A wrapper class for our custom-made operator board. Does not have a
 * constructor. Access statically. Access Buttons via public static state.
 * Access axis values via methods. Use methods to make buttons light up.\
 * Operator Board Buttons:
 * - Run Feeder - Button 8.
 * - Shooter On/Off - Button 9
 * - Arm down - Button 3
 * - Arm up - Button 4
 * - Intake - Button 2
 * - Intake Reverse - 1
 * - Climber Reset - 12
 */
public class OperatorBoard {

    private static Joystick joystick;
    public OperatorBoardButton intake;
    public OperatorBoardButton outtake;
    public OperatorBoardButton shoot;
    public OperatorBoardButton armUp;
    public OperatorBoardButton armDown;
    public OperatorBoardButton realignClimber;

    /**
     * Constructs one operator board. Since we only have one operator board, this
     * class should be treated as a singleton.
     * 
     * @param port The USB port number for the controller
     */
    public OperatorBoard(int port) {
        SmartDashboard.putNumber("constructor.called()", 1);

        joystick = new Joystick(port);
        
        intake = new OperatorBoardButton(OperatorButtonConstants.intakeButton);
        outtake = new OperatorBoardButton(OperatorButtonConstants.outtakeButton);
        shoot = new OperatorBoardButton(OperatorButtonConstants.shootButton);
        armUp = new OperatorBoardButton(OperatorButtonConstants.armUpButton);
        armDown = new OperatorBoardButton(OperatorButtonConstants.armDownButton);
        realignClimber = new OperatorBoardButton(OperatorButtonConstants.realignClimberButton);
    }

    /**
     * Access the x-axis of the left joystick.
     * 
     * @return Left is negative, right is positive.
     */
    public double getLeftJoystickX() {
        return joystick.getRawAxis(0);

    }

    /**
     * Access the y-axis of the left joystick.
     * 
     * @return Up is negative, down is positive.
     */
    public double getLeftJoystickY() {
        return joystick.getRawAxis(1);
    }

    /**
     * Access the x-axis of the right joystick.
     * 
     * @return Up is negative, down is positive.
     */
    public double getRightJoystickX() {
        return joystick.getRawAxis(2);
    }

    /**
     * Access the y-axis of the right joystick.
     * 
     * @return Up is negative, down is positive.
     */
    public double getRightJoystickY() {
        return joystick.getRawAxis(3);
    }

    /**
     * A custom button class for the operator board.
     */
    public class OperatorBoardButton extends Button {
        private int m_buttonNumber;

        /**
         * Constructs an OperatorBoardButton with a designated numerical ID
         * 
         * @param button The button's number - a sort of ID
         */
        public OperatorBoardButton(int button) {
            m_buttonNumber = button;
        }

        /**
         * Returns whether the button is being pressed.
         * 
         * @return Whether or not the button is being pressed.
         */
        @Override
        public boolean get() {
            return joystick.getRawButton(m_buttonNumber);
        }

        /**
         * Turns the button's light on, and keeps it on until turnLightOff() is called.
         */
        public void turnLightOn() {
            joystick.setOutput(m_buttonNumber, true);
            SmartDashboard.putNumber("light.on", 1);
        }

        /**
         * Turns the button's light off, and keeps it off until turnLightOn() is called.
         */
        public void turnLightOff() {
            joystick.setOutput(m_buttonNumber, false);
        }
    }
}