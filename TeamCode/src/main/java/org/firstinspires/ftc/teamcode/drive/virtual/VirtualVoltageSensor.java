package org.firstinspires.ftc.teamcode.drive.virtual;
import android.content.Context;

import com.qualcomm.hardware.R;
import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxUsbUtil;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class VirtualVoltageSensor implements VoltageSensor{
    public static final String TAG = "VirtualVoltageSensor";
    public VirtualVoltageSensor()
            throws RobotCoreException, InterruptedException
    {

    }

    //----------------------------------------------------------------------------------------------
    // HardwareDevice interface
    //----------------------------------------------------------------------------------------------
    protected String getTag() { return TAG; }
    public String getDeviceName()
    {
        return ("VirtualVoltageSensor");
    }

    //----------------------------------------------------------------------------------------------
    // VoltageSensor interface
    //----------------------------------------------------------------------------------------------

    @Override
    public double getVoltage()
    {
        //LynxGetADCCommand command = new LynxGetADCCommand(this.getModule(), LynxGetADCCommand.Channel.BATTERY_MONITOR, LynxGetADCCommand.Mode.ENGINEERING);
        //LynxGetADCResponse response = command.sendReceive();
        int mv = 10;//response.getValue();
        return mv * 0.001;

    }

    @Override
    public void close() {

    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }
}
