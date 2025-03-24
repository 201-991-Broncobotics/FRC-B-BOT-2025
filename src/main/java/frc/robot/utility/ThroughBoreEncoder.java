package frc.robot.utility;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ThroughBoreEncoder {

    private Encoder relativeEncoder;
    private DutyCycleEncoder absoluteEncoder;

    private final double relativeResolution = 8192;

    private double absoluteEncoderZero = 0, relativeEncoderZero = 0;

    public ThroughBoreEncoder(int channelA, int channelB, int channelC, boolean useAbolute, boolean useRelative) {

        // DigitalInput channelAPort = new DigitalInput(channelA);

        if (useAbolute) {
            absoluteEncoder = new DutyCycleEncoder(channelA);
        }
        if (useRelative) {
            relativeEncoder = new Encoder(channelB, channelC);
        }

    }

    public ThroughBoreEncoder(int channelA, int channelB, int channelC) {
        new ThroughBoreEncoder(channelA, channelB, channelC, true, true);
    }

    /**
     * For using only the duty cycle / absolute encoder
     * @param channelA
     */
    public ThroughBoreEncoder(int channelA) {
        new ThroughBoreEncoder(channelA, 0, 0, true, false);
    }

    /**
     * For using only the quadrature / relative encoder
     * @param channelB
     * @param channelC
     */
    public ThroughBoreEncoder(int channelB, int channelC) {
        new ThroughBoreEncoder(0, channelB, channelC, false, true);
    }

    public double getRelativeRaw() { return relativeEncoder.getRaw(); } // (relativeEncoder != null) ? relativeEncoder.getRaw() : 0.0;
    public double getAbsoluteRaw() { return absoluteEncoder.get(); } // return (absoluteEncoder != null) ? absoluteEncoder.getFrequency() : 0.0;

    public double getRelativeAngle() { return getRelativeRaw() / relativeResolution * 2*Math.PI - relativeEncoderZero; }
    public double getAbsoluteAngle() { return getAbsoluteRaw() * 2*Math.PI - absoluteEncoderZero; }

    public void resetRelative() { relativeEncoder.reset(); }
    public void setRelativeZero(double relativeZero) { relativeEncoderZero = relativeZero; }
    public void setAbsoluteZero(double absoluteZero) { absoluteEncoderZero = absoluteZero; }

    public boolean isAbsoluteConnected() { return absoluteEncoder.isConnected(); }

    public boolean encoderExists() { return this.relativeEncoder != null; }


}
