package frc.robot.utility;

public class CoralSystemPreset {
    public double eleHeight = 0;
    public double clawPitch = 0;
    public double clawRoll = 0;

    /**
     * Creates an object that contains all the preset variables for a coral system position
     * @param elevatorHeight inches
     * @param ClawPitch degrees (0° is straight)
     * @param ClawRoll degrees (0° is upright)
     */
    public CoralSystemPreset(double elevatorHeight, double ClawPitch, double ClawRoll) {
        eleHeight = elevatorHeight;
        clawPitch = Math.toRadians(ClawPitch);
        clawRoll = Math.toRadians(ClawRoll);
    }
}