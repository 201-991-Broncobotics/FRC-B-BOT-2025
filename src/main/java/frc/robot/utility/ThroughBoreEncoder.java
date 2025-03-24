package frc.robot.utility;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class ThroughBoreEncoder {

    private Encoder relativeEncoder;
    private DutyCycleEncoder absoluteEncoder;

    private final double relativeResolution = 8192;
    private final double absoluteResolution = 1024;

    private ThroughBoreEncoder(int channelA, int channelB, int channelC, boolean useAbolute, boolean useRelative) {
        
        if (useAbolute) {
            absoluteEncoder = new DutyCycleEncoder(new DigitalInput(channelA));
        }
        if (useRelative) {
            relativeEncoder = new Encoder(new DigitalInput(channelB), new DigitalInput(channelC));
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

    public double getRelativeRaw() { return relativeEncoder.getRaw(); }
    public double getAbsoluteRaw() { return absoluteEncoder.getFrequency(); }

    public double getRelativeAngle() { return relativeEncoder.getRaw() / relativeResolution * 2*Math.PI; }
    public double getAbsoluteAngle() { return (absoluteEncoder.getFrequency() - 1) / absoluteResolution * 2*Math.PI; }

    public void resetRelative() { relativeEncoder.reset(); }


}
