package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import java.io.File;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

public class PathXMLUtil {
    public static Pose2d[] parsePathXY(String filename) {
        String full_path = AppUtil.CONFIG_FILES_DIR + "/" + filename;
        String TAG = "PathXMLUtil";
        RobotLogger.dd(TAG, "path definition file: " + full_path);
        Pose2d[] coordinates = null;

        try {
            File inputFile = new File(AppUtil.CONFIG_FILES_DIR+"/"+filename);
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            Document doc = dBuilder.parse(inputFile);
            doc.getDocumentElement().normalize();
            RobotLogger.dd(TAG, "Root element :" + doc.getDocumentElement().getNodeName());
            NodeList nList = doc.getElementsByTagName("movestep");
            RobotLogger.dd(TAG,"----------------------------");
            int step_num = nList.getLength();
            RobotLogger.dd(TAG, "elements :" + Integer.toString(step_num));

            coordinates = new Pose2d[step_num];

            for (int temp = 0; temp < nList.getLength(); temp++) {
                Node nNode = nList.item(temp);
                //RobotLogger.dd(TAG, "Current Element :" + nNode.getNodeName());

                if (nNode.getNodeType() == Node.ELEMENT_NODE) {
                    double x, y, h;
                    Element eElement = (Element) nNode;
                    RobotLogger.dd(TAG, "step info : "
                            + eElement.getAttribute("drive_type"));
                    x = new Double(eElement
                            .getElementsByTagName("x")
                            .item(0)
                            .getTextContent());
                    y = new Double(eElement
                            .getElementsByTagName("y")
                            .item(0)
                            .getTextContent());
                    h = new Double(eElement
                            .getElementsByTagName("h")
                            .item(0)
                            .getTextContent());
                    RobotLogger.dd(TAG,"step %d: x: %f, y: %f, h: %f", temp, x, y , h);
                    coordinates[temp] = new Pose2d(x, y, Math.toRadians(h));
                }
            }
        } catch (Exception e) {
            RobotLogger.dd(TAG, "cannot find path XY file: " + full_path);
            e.printStackTrace();
        }
        return coordinates;
    }
}
