// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.Topic;

/** Add your docs here. */
public class PARTsNT {
    NetworkTableInstance nt_Instance = NetworkTableInstance.getDefault();
    NetworkTable table;

    List<String> topicsList;

    public PARTsNT() {
        table = nt_Instance.getTable(getClass().getSuperclass().getSimpleName());
    }

    public boolean topicExists(String name) {
        if (topicsList.contains(name)) return true;
        return false;
    }

    private void addToTList(String tName) {
        if (topicExists(tName)) return;
        topicsList.add(tName);
    }

    public void setDouble(double value, String name) {
        addToTList(name);
        DoublePublisher pub = table.getDoubleTopic(name).publish();
        pub.accept(value);
    }

    public double getDouble(String name) {
        DoubleSubscriber sub = table.getDoubleTopic(name).subscribe(0);
        return sub.get();
    }

    public void remove() {
        topicsList.removeAll(topicsList);
    }
}
