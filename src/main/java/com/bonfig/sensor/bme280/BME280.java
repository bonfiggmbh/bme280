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

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * BME280
 *
 * @author Dipl.-Ing. Robert C. Bonfig
 */
public class BME280 implements AutoCloseable {

    public enum Mode {
        SLEEP(0b00),
        FORCED(0b01),
        NORMAL(0b11);

        private final int value;

        Mode(int value) {
            this.value = value;
        }
    }

    public enum Sampling {
        NONE(0b000),
        X1(0b001),
        X2(0b010),
        X4(0b011),
        X8(0b100),
        X16(0b101);

        private final int value;

        Sampling(int value) {
            this.value = value;
        }
    }

    public enum Filter {
        OFF(0b000),
        X2(0b001),
        X4(0b010),
        X8(0b011),
        X16(0b100);

        private final int value;

        Filter(int value) {
            this.value = value;
        }
    }

    public enum StandbyTime {
        MS_0_5(0b000),
        MS_10(0b110),
        MS_20(0b111),
        MS_62_5(0b001),
        MS_125(0b010),
        MS_250(0b011),
        MS_500(0b100),
        MS_1000(0b101);

        private final int value;

        StandbyTime(int value) {
            this.value = value;
        }
    }

    public class Sample {

        private final float temperature;
        private final float pressure;
        private final float humidity;

        Sample(float temperature, float pressure, float humidity) {
            this.temperature = temperature;
            this.pressure = pressure;
            this.humidity = humidity;
        }

        public float getTemperature() {
            return temperature;
        }

        public float getPressure() {
            return pressure;
        }

        public float getHumidity() {
            return humidity;
        }

        @Override
        public String toString() {
            return String.format("temperature = %.2f°C, pressure = %.2fhPa, humidity = %.2f%%rH",
                    temperature, pressure / 100.0f, humidity);
        }
    }

    private static final int REGISTER_HUM_LSB = 0xFE;
    private static final int REGISTER_PRESS_MSB = 0xF7;
    private static final int REGISTER_CONFIG = 0xF5;
    private static final int REGISTER_CTRL_MEAS = 0xF4;
    private static final int REGISTER_STATUS = 0xF3;
    private static final int REGISTER_CTRL_HUM = 0xF2;
    private static final int REGISTER_RESET = 0xE0;
    private static final int REGISTER_ID = 0xD0;
    private static final int REGISTER_CALIB_41 = 0xF0;
    private static final int REGISTER_CALIB_26 = 0xE1;
    private static final int REGISTER_CALIB_25 = 0xA1;
    private static final int REGISTER_CALIB_00 = 0x88;
    private static final int VALUE_RESET = 0xB6;
    private static final int VALUE_ID = 0x60;
    private static final int STATUS_MASK_MEASURING = 0x08;
    private static final int STATUS_MASK_IM_UPDATE = 0x01;
    private Mode mode;
    private Sampling temperatureSampling;
    private Sampling pressureSampling;
    private Sampling humiditySampling;
    private Filter filter;
    private StandbyTime standbyTime;
    private int digT1;
    private int digT2;
    private int digT3;
    private int digP1;
    private int digP2;
    private int digP3;
    private int digP4;
    private int digP5;
    private int digP6;
    private int digP7;
    private int digP8;
    private int digP9;
    private int digH1;
    private int digH2;
    private int digH3;
    private int digH4;
    private int digH5;
    private int digH6;
    private final IO io;
    private final ByteBuffer buffer;

    public BME280() throws IOException {
        io = new I2CIO(I2CBus.BUS_1, 0x77);
        buffer = ByteBuffer.allocate(256);
        reset(Mode.FORCED, Sampling.X1, Sampling.X1, Sampling.X1, Filter.OFF, StandbyTime.MS_0_5);
    }

    @Override
    public void close() throws IOException {
        io.close();
    }

    public Mode getMode() {
        return mode;
    }

    public Sampling getTemperatureSampling() {
        return temperatureSampling;
    }

    public Sampling getPressureSampling() {
        return pressureSampling;
    }

    public Sampling getHumiditySampling() {
        return humiditySampling;
    }

    public Filter getFilter() {
        return filter;
    }

    public StandbyTime getStandbyTime() {
        return standbyTime;
    }

    public void reset(final Mode mode, final Sampling temperatureSampling, final Sampling pressureSampling,
                      final Sampling humiditySampling, final Filter filter, final StandbyTime standbyTime)
            throws IOException {
        this.mode = mode;
        this.pressureSampling = pressureSampling;
        this.temperatureSampling = temperatureSampling;
        this.humiditySampling = humiditySampling;
        this.filter = filter;
        this.standbyTime = standbyTime;

        // Check device id
        if (io.read(REGISTER_ID) != VALUE_ID) {
            throw new IOException("Device id wrong: " + io.read(REGISTER_ID) + " != " + VALUE_ID);
        }

        // Perform a soft reset
        io.write(REGISTER_RESET, VALUE_RESET);
        // Await reset to complete ...
        wait(300);
        // ... and calibration data to be copied
        while ((io.read(REGISTER_STATUS) & STATUS_MASK_IM_UPDATE) != 0) {
            wait(100);
        }

        // Read calibration data
        buffer.clear();
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        buffer.limit(REGISTER_CALIB_25 - REGISTER_CALIB_00 + 1);
        io.read(REGISTER_CALIB_00, buffer);
        assert buffer.remaining() == 0;
        buffer.limit(buffer.limit() + REGISTER_CALIB_41 - REGISTER_CALIB_26 + 1);
        io.read(REGISTER_CALIB_26, buffer);
        assert buffer.remaining() == 0;
        buffer.flip();
        digT1 = 0xFFFF & buffer.getShort();
        digT2 = buffer.getShort();
        digT3 = buffer.getShort();
        digP1 = 0xFFFF & buffer.getShort();
        digP2 = buffer.getShort();
        digP3 = buffer.getShort();
        digP4 = buffer.getShort();
        digP5 = buffer.getShort();
        digP6 = buffer.getShort();
        digP7 = buffer.getShort();
        digP8 = buffer.getShort();
        digP9 = buffer.getShort();
        buffer.get(); // unused
        digH1 = 0xFF & buffer.get();
        digH2 = buffer.getShort();
        digH3 = 0xFF & buffer.get();
        digH4 = buffer.get() << 4 | 0x0F & buffer.get();
        digH5 = 0x0F & buffer.get() >> 4 | buffer.get() << 4;
        digH6 = buffer.get();
        assert buffer.remaining() == 8; // unused

        // Set configuration. REGISTER_CTRL_HUM must be followed by REGISTER_CTRL_MEAS to be written.
        buffer.clear();
        buffer.order(ByteOrder.BIG_ENDIAN);
        buffer.put((byte) REGISTER_CONFIG).put((byte) getConfigValue());
        buffer.put((byte) REGISTER_CTRL_HUM).put((byte) getCtrlHumValue());
        buffer.put((byte) REGISTER_CTRL_MEAS).put((byte) getCtrlMeasValue());
        buffer.flip();
        io.write(buffer);
        assert buffer.remaining() == 0;
    }

    public Sample read() throws IOException {
        if (mode == Mode.FORCED) {
            // Trigger forced mode sample
            buffer.clear();
            buffer.order(ByteOrder.BIG_ENDIAN);
            buffer.put((byte) REGISTER_CTRL_MEAS).put((byte) getCtrlMeasValue());
            buffer.flip();
            io.write(buffer);
            assert buffer.remaining() == 0;

            while ((io.read(REGISTER_STATUS) & STATUS_MASK_MEASURING) != 0) {
                wait(1);
            }
        }

        // Read sample
        buffer.clear();
        buffer.order(ByteOrder.BIG_ENDIAN);
        buffer.limit(REGISTER_HUM_LSB - REGISTER_PRESS_MSB + 1);
        io.read(REGISTER_PRESS_MSB, buffer);
        assert buffer.remaining() == 0;
        buffer.flip();
        int adcP = buffer.getShort() << 4 | 0x0F & buffer.get();
        int adcT = buffer.getShort() << 4 | 0x0F & buffer.get();
        int adcH = buffer.getShort();
        // System.out.printf("adcT = %d, adcP = %d, adcH = %d%n", adcT, adcP, adcH);
        assert buffer.remaining() == 0;

        return compensate(adcT, adcP, adcH);
    }

    public static void main(String ... args) throws IOException {
        try(BME280 bme280 = new BME280()) {
            Sample sample = bme280.read();
            System.out.println(sample);
        }
    }

    private int getConfigValue() {
        return this.standbyTime.value << 5 | this.filter.value << 3;
    }

    private int getCtrlHumValue() {
        return this.humiditySampling.value;
    }

    private int getCtrlMeasValue() {
        return this.temperatureSampling.value << 5 | this.pressureSampling.value << 3 | this.mode.value;
    }

    Sample compensate(int adcT, int adcP, int adcH) {
        int tFine = 0;
        float temperature = Float.NaN;
        float pressure = Float.NaN;
        float humidity = Float.NaN;

        if (adcT != 0xFFF80000) {
            int t1 = ((((adcT >> 3) - ((int) digT1 << 1))) * ((int) digT2)) >> 11;
            int t2 = (((((adcT >> 4) - ((int) digT1)) * ((adcT >> 4) - ((int) digT1))) >> 12) *
                    ((int) digT3)) >> 14;
            tFine = t1 + t2;
            int t3 = (tFine * 5 + 128) >> 8;
            temperature = (float) t3 / 100.0f;

            if (adcP != 0xFFF80000) {
                long p1 = ((long) tFine) - 128000;
                long p2 = p1 * p1 * (long) digP6;
                p2 = p2 + ((p1 * (long) digP5) << 17);
                p2 = p2 + (((long) digP4) << 35);
                p1 = ((p1 * p1 * (long) digP3) >> 8) + ((p1 * (long) digP2) << 12);
                p1 = (((((long) 1) << 47) + p1)) * ((long) digP1) >> 33;
                if (p1 != 0) {
                    long p3 = 1048576 - adcP;
                    p3 = (((p3 << 31) - p2) * 3125) / p1;
                    p1 = (((long) digP9) * (p3 >> 13) * (p3 >> 13)) >> 25;
                    p2 = (((long) digP8) * p3) >> 19;
                    p3 = ((p3 + p1 + p2) >> 8) + (((long) digP7) << 4);
                    pressure = (float) p3 / 256.0f;
                }
            }

            if (adcH != 0xFFFF8000) {
                int h1 = (tFine - ((int) 76800));
                h1 = (((((adcH << 14) - (((int) digH4) << 20) - (((int) digH5) * h1)) +
                        ((int) 16384)) >> 15) * (((((((h1 * ((int) digH6)) >> 10) * (((h1 *
                        ((int) digH3)) >> 11) + ((int) 32768))) >> 10) + ((int) 2097152)) *
                        ((int) digH2) + 8192) >> 14));
                h1 = (h1 - (((((h1 >> 15) * (h1 >> 15)) >> 7) * ((int) digH1)) >> 4));
                h1 = (h1 < 0 ? 0 : h1);
                h1 = (h1 > 419430400 ? 419430400 : h1);
                humidity = (float) (h1 >> 12) / 1024.0f;
            }
        }
        
        return new Sample(temperature, pressure, humidity);
    }

    private void wait(int millis) {
        try {
            Thread.sleep(millis, 0);
        } catch (InterruptedException e) {
            // ignore
        }
    }

}
