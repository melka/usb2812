/*
 *  usb2812 java example
 *  -------------------------
 *  Connect to Resolume Arena via Syphon, and use the received video
 *  to update a 10x10 WS2812B matrix connected to USB via a Sparkfun
 *  Pro Micro.
 *  Uses processing (processing.org), usb4java (http://usb4java.org/)
 *  and Syphon (https://github.com/Syphon/Processing).
 */

import java.nio.ByteBuffer;

import org.usb4java.Context;
import org.usb4java.Device;
import org.usb4java.DeviceDescriptor;
import org.usb4java.DeviceHandle;
import org.usb4java.DeviceList;
import org.usb4java.LibUsb;
import org.usb4java.LibUsbException;

import processing.core.PApplet;
import processing.core.PGraphics;
import processing.core.PImage;
import codeanticode.syphon.SyphonClient;

public class usb2812 extends PApplet {

	Context context;
	Device device;
	DeviceHandle handle;
	int LEDS = 107;
	byte[] message = new byte[LEDS*3];
	PGraphics canvas;
	SyphonClient client;

	public void setup() {
		size(200,200,P3D);
		registerMethod("dispose",this);
		client = new SyphonClient(this, "Arena", "Composition");
		int result = LibUsb.init(null);
    if (result != LibUsb.SUCCESS) {
      throw new LibUsbException("Unable to initialize libusb", result);
    }

    // Search for the device
    device = findDevice((short)0xABCD,(short)0xF003);
    if (device == null) {
      System.err.println("usb2812 not found.");
      System.exit(1);
    }

    // Open the device
    handle = new DeviceHandle();
    result = LibUsb.open(device, handle);
    if (result != LibUsb.SUCCESS) {
      throw new LibUsbException("Unable to open USB device", result);
    } try {
      // Claim interface
      result = LibUsb.claimInterface(handle, 0);
      if (result != LibUsb.SUCCESS) {
        throw new LibUsbException("Unable to claim interface", result);
      }
    } finally {
      // DO NOTHING
    }
    colorMode(HSB,1);
	}

	public static void sendMessage(DeviceHandle handle, byte[] message) {
    ByteBuffer buffer = ByteBuffer.allocateDirect(message.length);
    buffer.put(message);
    buffer.rewind();
    int transfered = LibUsb.controlTransfer(handle, (byte) 0x40,  (byte) 0xA5, (short) 0x0100, (short) 0x0000, buffer, 1000);
    if (transfered < 0)
        throw new LibUsbException("Control transfer failed", transfered);
    if (transfered != message.length)
        throw new RuntimeException("Not all data was sent to device");
  }

	public Device findDevice(short vendorId, short productId) {
    // Read the USB device list
    DeviceList list = new DeviceList();
    int result = LibUsb.getDeviceList(null, list);
    if (result < 0) throw new LibUsbException("Unable to get device list", result);

    try {
      // Iterate over all devices and scan for the right one
      for (Device device: list) {
        DeviceDescriptor descriptor = new DeviceDescriptor();
        result = LibUsb.getDeviceDescriptor(device, descriptor);
        if (result != LibUsb.SUCCESS) throw new LibUsbException("Unable to read device descriptor", result);
        if (descriptor.idVendor() == vendorId && descriptor.idProduct() == productId) return device;
      }
    } finally {
      // Ensure the allocated device list is freed
      LibUsb.freeDeviceList(list, true);
    }

    // Device not found
    return null;
	}

	public void draw() {
		background(0);
		if (client.available()) {
			canvas = client.getGraphics(canvas);
			PImage frame = canvas.get(0, 0, 10, 11);
			sendFrame (frame);
			image(frame, 0, 0);
		}
	}

	public void sendFrame (PImage img) {
		img.loadPixels();
		for (int i = 0;i<LEDS;i++) {
			int c = img.pixels[i];
			int r = HSVtoHEX(hue(c),saturation(c),brightness(c)*0.2f);
    	message[i*3+1] = (byte) (r & 0xff);
    	message[i*3+0] = (byte) ((r>>8) & 0xff);
    	message[i*3+2] = (byte) ((r>>16) & 0xff);
    }
		img.updatePixels();
    sendMessage(handle, message);
	}

	public int HSVtoHEX(float hue, float sat, float value ) {
		float pr = 0;
		float pg = 0;
		float pb = 0;

		short ora = 0;
		short og = 0;
		short ob = 0;

		float ro = (hue * 6) % 6.0f;

		float avg = 0;

		ro = (ro + 6 + 1) % 6.0f; //Hue was 60* off...

		if( ro < 1 ) //yellow->red
		{
			pr = 1;
			pg = 1.0f - ro;
		} else if( ro < 2 )
		{
			pr = 1;
			pb = ro - 1.0f;
		} else if( ro < 3 )
		{
			pr = 3.0f - ro;
			pb = 1;
		} else if( ro < 4 )
		{
			pb = 1;
			pg = ro - 3;
		} else if( ro < 5 )
		{
			pb = 5 - ro;
			pg = 1;
		} else
		{
			pg = 1;
			pr = ro - 5;
		}

		//Actually, above math is backwards, oops!
		pr *= value;
		pg *= value;
		pb *= value;

		avg += pr;
		avg += pg;
		avg += pb;

		pr = pr * sat + avg * (1.0f-sat);
		pg = pg * sat + avg * (1.0f-sat);
		pb = pb * sat + avg * (1.0f-sat);

		ora = (short) (pr * 255.0f);
		og = (short) (pb * 255.0f);
		ob = (short) (pg * 255.0f);

		if( ora < 0 ) ora = 0;
		if( ora > 255 ) ora = 255;
		if( og < 0 ) og = 0;
		if( og > 255 ) og = 255;
		if( ob < 0 ) ob = 0;
		if( ob > 255 ) ob = 255;

		return (ob<<16) | (og<<8) | ora;
	}

	public void dispose() {
		int result = LibUsb.releaseInterface(handle, 0);
    if (result != LibUsb.SUCCESS) {
      throw new LibUsbException("Unable to release interface", result);
    }
		LibUsb.close(handle);
		LibUsb.exit(null);
  }
}
