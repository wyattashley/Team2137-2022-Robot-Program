package frc.robot.functions.io.xmlreader;

import frc.robot.library.CANDevice;
import org.w3c.dom.Element;

public class Encoder extends CANDevice {

    public enum EncoderTypes {
        CTRE_CAN_ABS ("CTRE_CAN_ABS");

        final String name;

        EncoderTypes (String value) {
            name = value;
        }

        @Override
        public String toString() {
            return name;
        }
    }

    EncoderTypes type;
    boolean inverted;
    double offset;
    String[] parms;

    public Encoder(String _name, int _id, EncoderTypes _type, boolean _invert, double offset, String... _parms) {
        super(_name, _id);
        this.type = _type;
        this.inverted = _invert;
        this.offset = offset;
        this.parms = _parms;
    }

    public Encoder(Element element) {
        super(element);

        this.type = EncoderTypes.valueOf(getOrDefault(element, "Type", "CTRE_CAN_ABS").toUpperCase());
        this.inverted = Boolean.parseBoolean(getOrDefault(element, "Inverted", "false").toLowerCase());
        this.offset = Double.parseDouble(getOrDefault(element, "Offset", "0"));
    }

    public EncoderTypes getEncoderType() {
        return this.type;
    }

    public String[] getParms() {
        return this.parms;
    }

    public String getParm(int i) {
        return this.parms[i];
    }

    public boolean inverted() {
        return this.inverted;
    }

    public double getOffset() {
        return offset;
    }

    @Override
    public String toString() {
        return super.toString() + "\nType: " + type.toString() + "\nInverted: " + inverted + "\nOffset: " + offset;
    }
}
