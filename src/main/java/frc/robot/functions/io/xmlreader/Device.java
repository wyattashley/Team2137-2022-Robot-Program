package frc.robot.functions.io.xmlreader;

import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class Device {
    private String strName;

    public Device(String name) {
        strName = name;
    }

    public Device(Element element) {
        strName = getOrDefault(element, "Name", "Default");
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
}

