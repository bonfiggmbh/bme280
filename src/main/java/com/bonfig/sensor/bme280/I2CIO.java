/*
 * Copyright 2017 Bonfig GmbH
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.bonfig.sensor.bme280;

import com.pi4j.io.i2c.I2CBus;
import com.pi4j.io.i2c.I2CDevice;
import com.pi4j.io.i2c.I2CFactory;

import java.io.IOException;
import java.nio.ByteBuffer;

/**
 * I2CIO
 *
 * @author Dipl.-Ing. Robert C. Bonfig
 */
public class I2CIO implements IO {

    private final I2CBus i2cBus;
    private final I2CDevice i2cDevice;
    private final byte[] readBuffer;
    private final byte[] writeBuffer;

    public I2CIO(int busNumber, int address) throws IOException {
        try {
            i2cBus = I2CFactory.getInstance(busNumber);
            i2cDevice = i2cBus.getDevice(address);
            readBuffer = new byte[256];
            writeBuffer = new byte[256];
        } catch (I2CFactory.UnsupportedBusNumberException e) {
            throw new IOException(e);
        }
    }

    @Override
    public void close() throws IOException {
        i2cBus.close();
    }

    @Override
    public int read(int address) throws IOException {
        return i2cDevice.read(address);
    }

    @Override
    public void read(int address, ByteBuffer buffer) throws IOException {
        int length = Math.min(buffer.remaining(), readBuffer.length);
        for (int n = 0; n < length;) {
            n += i2cDevice.read(address + n, readBuffer, n, length - n);
        }
        buffer.put(readBuffer, 0, length);
    }

    @Override
    public void write(int address, int value) throws IOException {
        i2cDevice.write(address, (byte) value);
    }

    @Override
    public void write(ByteBuffer buffer) throws IOException {
        int length = Math.min(buffer.remaining(), writeBuffer.length);
        if (length > 0) {
            buffer.get(writeBuffer, 0, length);
            i2cDevice.write(writeBuffer, 0, length);
        }
    }

}
