package frc.robot;

public abstract class Helpers {
    /**
     * Formats the joystick input using OperatorConstants.joystickDeadband and with a cubic function *DO NOT USE FOR SWERVE
     * @param rawIn the raw joystick input
     * @return formated joystick input
     */
    public static double formatJoyInput(double rawIn) {
        return Math.pow((rawIn - Constants.OperatorConstants.joystickDeadband * (rawIn > 0 ? 1 : -1)) / (1 - Constants.OperatorConstants.joystickDeadband), 3);
    }
}
