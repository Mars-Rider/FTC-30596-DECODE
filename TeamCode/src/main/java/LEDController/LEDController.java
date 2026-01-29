package LEDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import java.util.ArrayList;
import java.util.List;

public class LEDController {

    final I2cDeviceSynch device;
    final List<LEDChannel> channels = new ArrayList<>();

    private String SSID = "LED Controller";

    public enum Strip {
        WS2812B(1), SK6812(2);

        public final int id;
        Strip(int id){ this.id=id; }
    }

    LEDController(HardwareMap hardwareMap,String deviceName){
        device=hardwareMap.get(I2cDeviceSynch.class,deviceName); device.engage();
    }

    public LEDChannel addChannel() {
        if (channels.size() >= 5) throw new IllegalStateException("Maximum of 5 LED channels allowed");
        return addChannel(Strip.WS2812B, 30);
    }

    public LEDChannel addChannel(int ledsNum) {
        if (channels.size() >= 5) throw new IllegalStateException("Maximum of 5 LED channels allowed");
        return addChannel(Strip.WS2812B, ledsNum);
    }

    public LEDChannel addChannel(Strip s, int ledsNum) {
        if (channels.size() >= 5) throw new IllegalStateException("Maximum of 5 LED channels allowed");
        LEDChannel ch = new LEDChannel(this, s);
        channels.add(ch);

        byte[] packet=new byte[4];
        packet[0]=(byte)ch.getChannelIndex();
        packet[1]=(byte)(ch.type.id);
        packet[2]=(byte)((ledsNum >> 8) & 0xFF);
        packet[3]=(byte)(ledsNum & 0xFF);
        device.write(0x00,packet);
        return ch;
    }

    public void setSSID(String ssid){
        SSID = ssid;
    }

    public void startWiFi(){
        startWiFi(SSID);
    }

    public void startWiFi(String ssid){
        if(SSID != ssid){SSID = ssid;}

        byte[] data = new byte[32];
        byte[] ssidBytes = ssid.getBytes(); // UTF-8 / ASCII

        // Copy string, pad with spaces
        for (int i = 0; i < 32; i++){
            if (i < ssidBytes.length) {
                data[i] = ssidBytes[i];
            } else {
                data[i] = ' '; // pad remaining bytes
            }
        }

        // Send to Arduino over I2C
        device.write(0x00, data);
    }


    //Update all data
    public void update(){
        for(LEDChannel ch:channels){
            ch.update();
        }
    }
}
