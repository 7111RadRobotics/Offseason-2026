package team7111.robot.utils.encoder;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.EncoderSim;

public class SimDIOEncoder implements GenericEncoder {

    private EncoderSim encoderSim;
    private org.wpilib.hardware.rotation.Encoder encoder;

    private SimDIOEncoder(int channel){
        encoder = new org.wpilib.hardware.rotation.Encoder(channel, channel + 1);
        encoderSim = new EncoderSim(encoder);
    }

    @Override
    public Rotation2d getPosition() {
        return Rotation2d.fromDegrees(encoder.getDistance());
    }

    @Override
    public void setPosition(Rotation2d rotation) {
        encoderSim.setDistance(rotation.getRotations());
    }

}
