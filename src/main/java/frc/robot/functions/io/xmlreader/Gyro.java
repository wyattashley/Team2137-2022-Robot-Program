package frc.robot.functions.io.xmlreader;

import frc.robot.library.CANDevice;
import org.w3c.dom.Element;

public class Gyro extends CANDevice {

    public enum GyroTypes {
        PIGEON ("PIGEON");

        final String name;

        GyroTypes (String value) {
            name = value;
        }

        @Override
        public String toString() {
            return name;
        }
    }

    GyroTypes type;
    boolean inverted;
    double offset;
    String[] parms;

    public Gyro(String _name, int _id, GyroTypes _type, boolean _invert, double offset, String... _parms) {
        super(_name, _id);
        this.type = _type;
        this.inverted = _invert;
        this.offset = offset;
        this.parms = _parms;
    }

    public Gyro(Element element) {
        super(element);

        this.type = GyroTypes.valueOf(getOrDefault(element, "Type", "Pigeon"));
        this.inverted = Boolean.parseBoolean(getOrDefault(element, "Inverted", "false"));
        this.offset = Double.parseDouble(getOrDefault(element, "Offset", "0"));
    }

    public GyroTypes getEncoderType() {
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
