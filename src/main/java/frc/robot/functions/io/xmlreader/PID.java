package frc.robot.functions.io.xmlreader;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import org.w3c.dom.Element;

public class PID extends Device {

    private double P, I, D, IZ, FF = 0;
    private double S, V, A = 0;

    public PID (double _P, double _I, double _D, String name) {
        super(name);
        P = _P;
        I = _I;
        D = _D;
    }

    public PID (Double _P, Double _I, Double _D, Double _FF, Double _IZ, String name) {
        this(_P, _I, _D, name);
        IZ = _IZ;
        FF = _FF;
    }

    public PID (Double _P, Double _I, Double _D, Double _S, Double _V, Double _A, String name) {
        this(_P, _I, _D, name);
        S = _S;
        V = _V;
        A = _A;
    }

    public PID (Double _P, Double _I, Double _D, Double _FF, Double _IZ, Double _S, Double _V, Double _A, String name) {
        this(_P, _I, _D, _S, _V, _A, name);
        FF = _FF;
        IZ = _IZ;
    }

    public PID(Element element) {
        super(element);

        if (element != null) {
            var nodes = element.getElementsByTagName("P");
            if (nodes.getLength() > 0)
                P = Double.parseDouble(nodes.item(0).getTextContent());

            nodes = element.getElementsByTagName("I");
            if (nodes.getLength() > 0)
                I = Double.parseDouble(nodes.item(0).getTextContent());

            nodes = element.getElementsByTagName("D");
            if (nodes.getLength() > 0)
                D = Double.parseDouble(nodes.item(0).getTextContent());

            nodes = element.getElementsByTagName("IZ");
            if (nodes.getLength() > 0)
                IZ = Double.parseDouble(nodes.item(0).getTextContent());

            nodes = element.getElementsByTagName("FF");
            if (nodes.getLength() > 0)
                FF = Double.parseDouble(nodes.item(0).getTextContent());

            nodes = element.getElementsByTagName("S");
            if (nodes.getLength() > 0)
                S = Double.parseDouble(nodes.item(0).getTextContent());

            nodes = element.getElementsByTagName("V");
            if (nodes.getLength() > 0)
                V = Double.parseDouble(nodes.item(0).getTextContent());

            nodes = element.getElementsByTagName("A");
            if (nodes.getLength() > 0)
                A = Double.parseDouble(nodes.item(0).getTextContent());
        }
    }

    public double getP() {
        return P;
    }

    public void setP(Double p) {
        P = p;
    }

    public double getI() {
        return I;
    }

    public void setI(Double i) {
        I = i;
    }

    public double getD() {
        return D;
    }

    public void setD(Double d) {
        D = d;
    }

    public double getIZ() {
        return IZ;
    }

    public void setIZ(Double IZ) {
        IZ = IZ;
    }

    public double getFF() {
        return FF;
    }

    public void setFF(Double FF) {
        FF = FF;
    }

    public double getS() {
        return S;
    }

    public void setS(Double s) {
        S = s;
    }

    public double getV() {
        return V;
    }

    public void setV(Double v) {
        V = v;
    }

    public double getA() {
        return A;
    }

    public void setA(Double a) {
        A = a;
    }

    public Double[] getPIDArray() {
        return new Double[] {P, I, D};
    }

    public PIDController getWPIPIDController() {
        return new PIDController(P, I, D);
    }

    public SimpleMotorFeedforward getWPIFeedForwardController() {
        return new SimpleMotorFeedforward(S, V, A);
    }

    public void setREVPIDValues(CANPIDController controller) {
        controller.setP(P);
        controller.setI(I);
        controller.setD(D);

        controller.setFF(FF);
    }

    @Override
    public String toString() {
        return "P: " + P + " I: " + I + " D: " + D;
    }
}
