package com.iamverylovely;

import android.os.ParcelFileDescriptor;

import java.io.File;
import java.io.IOException;

public class ASerialPort {

    public ASerialPort(String path, int baudRate, IDataCallback _callback) {
        System.loadLibrary("comm1");
        try (ParcelFileDescriptor pFd = ParcelFileDescriptor.open(new File(path), ParcelFileDescriptor.MODE_READ_WRITE)) {
            reading(pFd.detachFd(), baudRate, _callback);
            if (_callback != null)
                _callback.OnError(0);
        } catch (IOException ignored) {
            if (_callback != null)
                _callback.OnError(1);
        }
    }

    public native void send(byte[] data);

    public native void close();

    private native int reading(int fd, int speed, IDataCallback _callback);

    public interface IDataCallback {
        void OnReceive(byte[] data, int len);

        void OnError(int code);
    }
}