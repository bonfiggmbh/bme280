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

import java.io.IOException;
import java.nio.ByteBuffer;

/**
 * IO
 *
 * @author Dipl.-Ing. Robert C. Bonfig
 */
interface IO extends AutoCloseable {

    void close() throws IOException;

    int read(int address) throws IOException;

    void read(int address, ByteBuffer buffer) throws IOException;

    void write(int address, int value) throws IOException;

    void write(ByteBuffer buffer) throws IOException;

}
