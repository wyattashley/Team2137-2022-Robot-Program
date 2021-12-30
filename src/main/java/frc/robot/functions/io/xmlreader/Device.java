package frc.robot.functions.io.xmlreader;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

//public class Device extends SubsystemBase {
public class Device {

    private String strName;

    public Device(String name) {
        strName = name;
        Robot.deviceCallList.add(this);
    }

    public Device(Element element) {
        this(getOrDefault(element, "Name", "Default"));
    }

    protected static String getOrDefault(Element element, String name, String defaultReturn) {
        NodeList list = element.getElementsByTagName(name);
        if(list.getLength() > 0)
            return list.item(0).getTextContent();
        else
            return defaultReturn;
    }

    public String getName() {
        return strName;
    }

    public void setName(String strName) {
        this.strName = strName;
    }

    @Override
    public String toString() {
        return "Name: " + strName;
    }

    public void periodic() {
        Robot.deviceCallList.remove(this);
    }
}

