#include "SequencerBatch.h"

// -----------------------------------------------------------------------
// Construction / reset
// -----------------------------------------------------------------------
SequencerBatch::SequencerBatch() {
    clear();
}

void SequencerBatch::clear() {
    seq.clear();
    readCount    = 0;
    sramCursor   = 0;
    accumDelayMs = 0;
    executed     = false;
    resultLen    = 0;
    memset(reads, 0, sizeof(reads));
}

// -----------------------------------------------------------------------
// Delay-value helper
// DS28E18 delay opcode: 0xDD <TVAL>
// The delay is encoded in powers of two milliseconds:
//   TVAL=0 -> 1ms, TVAL=1 -> 2ms, TVAL=2 -> 4ms, ...
// So we round the requested delay up to the next supported step.
// -----------------------------------------------------------------------
uint8_t SequencerBatch::msToDelayVal(uint16_t ms) {
    if (ms <= 1) return 0;

    uint8_t val = 0;
    uint16_t actual = 1;
    while (actual < ms && val < 15) {
        actual <<= 1;
        val++;
    }
    return val;
}

// -----------------------------------------------------------------------
// addI2CWrite  —  START + WRITE(addr, data) + STOP
// -----------------------------------------------------------------------
bool SequencerBatch::addI2CWrite(uint8_t i2cAddr,
                                  const uint8_t *data, uint8_t dataLen)
{
    // Bytes needed: START(1) + WRITE(2+1+dataLen) + STOP(1)
    uint16_t needed = 1 + (2 + 1 + dataLen) + 1;
    if (!seq.wouldFit(needed)) return false;

    seq.addStart();                          // 1 byte
    sramCursor += 1;

    seq.addWrite(i2cAddr, data, dataLen);    // 2 + 1 + dataLen bytes
    sramCursor += 2 + 1 + dataLen;

    seq.addStop();                           // 1 byte
    sramCursor += 1;

    return true;
}

// -----------------------------------------------------------------------
// addI2CRead  —  START + WRITE(addr+R) + READ_NACK(readLen) + STOP
// -----------------------------------------------------------------------
int8_t SequencerBatch::addI2CRead(uint8_t i2cAddr, uint16_t readLen) {
    if (readCount >= BATCH_MAX_READS) return -1;
    if (readLen == 0 || readLen > 255) return -1;

    // Bytes needed: START(1) + WRITE_ADDR_R(3) + READ_NACK(2+readLen) + STOP(1)
    uint16_t needed = 1 + 3 + (2 + readLen) + 1;
    if (!seq.wouldFit(needed)) return -1;

    seq.addStart();                     // 1 byte
    sramCursor += 1;

    seq.addRead(i2cAddr, readLen);      // addRead internally does:
    // WRITE(3 bytes: 0xE3, 1, addr|1) + READ_NACK(2+readLen bytes: 0xD3, len, FF...)
    sramCursor += 3;                    // write header for addr+R

    // Now the read data starts at current sramCursor + 2 (opcode + len byte)
    uint16_t dataOffset = sramCursor + 2;
    sramCursor += 2 + readLen;

    seq.addStop();                      // 1 byte
    sramCursor += 1;

    // Record the read slot
    int8_t handle = (int8_t)readCount;
    reads[readCount].sramOffset = dataOffset;
    reads[readCount].dataLen    = readLen;
    readCount++;

    return handle;
}

// -----------------------------------------------------------------------
// addI2CWriteRead  —  write + delay + read combined
// -----------------------------------------------------------------------
int8_t SequencerBatch::addI2CWriteRead(uint8_t i2cAddr,
                                         const uint8_t *writeData,
                                         uint8_t writeLen,
                                         uint16_t readLen,
                                         uint16_t delayMs)
{
    if (readCount >= BATCH_MAX_READS) return -1;
    if (readLen == 0 || readLen > 255) return -1;

    // Calculate total bytes needed:
    // START(1) + WRITE(2+1+writeLen) + STOP(1) + DELAY(2) +
    // START(1) + WRITE_ADDR_R(3) + READ_NACK(2+readLen) + STOP(1)
    uint16_t needed = 1 + (2 + 1 + writeLen) + 1;
    if (delayMs > 0) needed += 2;
    needed += 1 + 3 + (2 + readLen) + 1;
    if (!seq.wouldFit(needed)) return -1;

    // --- Write phase ---
    seq.addStart();
    sramCursor += 1;

    seq.addWrite(i2cAddr, writeData, writeLen);
    sramCursor += 2 + 1 + writeLen;

    seq.addStop();
    sramCursor += 1;

    // --- Delay phase ---
    if (delayMs > 0) {
        uint8_t delayVal = msToDelayVal(delayMs);
        seq.addDelay(delayVal);
        sramCursor += 2;
        accumDelayMs += (uint16_t)1 << delayVal;
    }

    // --- Read phase ---
    seq.addStart();
    sramCursor += 1;

    seq.addRead(i2cAddr, readLen);
    sramCursor += 3;  // write header (0xE3, 1, addr|1)

    uint16_t dataOffset = sramCursor + 2;  // skip 0xD3 + len byte
    sramCursor += 2 + readLen;

    seq.addStop();
    sramCursor += 1;

    // Record the read slot
    int8_t handle = (int8_t)readCount;
    reads[readCount].sramOffset = dataOffset;
    reads[readCount].dataLen    = readLen;
    readCount++;

    return handle;
}

// -----------------------------------------------------------------------
// addDelay  —  add a standalone delay to the sequence
// -----------------------------------------------------------------------
bool SequencerBatch::addDelay(uint16_t delayMs) {
    if (!seq.wouldFit(2)) return false;
    uint8_t delayVal = msToDelayVal(delayMs);
    seq.addDelay(delayVal);
    sramCursor   += 2;
    accumDelayMs += (uint16_t)1 << delayVal;
    return true;
}

// -----------------------------------------------------------------------
// execute  —  write → run → readback
// -----------------------------------------------------------------------
bool SequencerBatch::execute(DS28E18 &ds, uint16_t tOP_extra_ms) {
    executed = false;
    uint16_t seqLen = seq.getLength();
    if (seqLen == 0) return false;

    // writeSequencer supports max 128 bytes per call.
    // For longer sequences we need to write in chunks.
    uint8_t *buf = seq.getBuffer();
    uint16_t written = 0;
    while (written < seqLen) {
        uint16_t chunk = seqLen - written;
        if (chunk > 128) chunk = 128;
        if (!ds.writeSequencer(written, buf + written, chunk)) {
            DBG_PRINTLN("SequencerBatch: writeSequencer failed");
            return false;
        }
        written += chunk;
    }

    // Run with SPU; tOP must cover the full sequencer execution time.
    // accumDelayMs tracks actual hardware delay (TVAL+1 ms per delay opcode).
    // Add 50% margin for I2C transaction overhead + extra user margin.
    uint16_t tOP = accumDelayMs + (accumDelayMs / 2) + tOP_extra_ms;
    if (tOP < 10) tOP = 10;  // minimum safe delay
    uint8_t result;
    if (!ds.runSequencer(0, seqLen, result, tOP)) {
        DBG_PRINTLN("SequencerBatch: runSequencer failed");
        return false;
    }

    // Read back — readSequencer returns [result_byte] + [sram data].
    // We do this EVEN IF the sequence failed (result != 0xAA) so we can 
    // inspect the SRAM to see exactly where the NACK occurred.
    resultLen = 0;
    uint16_t toRead = seqLen;
    uint16_t sramAddr = 0;     // SRAM address to read from
    uint16_t destPos  = 1;     // resultBuf[0] reserved for result byte

    // First chunk: keep the result byte
    {
        uint16_t chunk = (toRead > 128) ? 128 : toRead;
        uint16_t outLen = 0;
        uint8_t tmpBuf[140];
        if (!ds.readSequencer(0, tmpBuf, chunk, outLen)) {
            DBG_PRINTLN("SequencerBatch: readSequencer failed (chunk 0)");
            return false;
        }
        // tmpBuf[0] = result byte, tmpBuf[1..outLen-1] = sram data
        resultBuf[0] = tmpBuf[0];  // keep result byte
        uint16_t dataBytes = (outLen > 1) ? outLen - 1 : 0;
        if (dataBytes > 0) {
            memcpy(resultBuf + destPos, tmpBuf + 1, dataBytes);
            destPos += dataBytes;
        }
        sramAddr += (dataBytes);  // we've read this many SRAM bytes
    }

    // Subsequent chunks (if sequence > 128 bytes)
    while (sramAddr < toRead) {
        uint16_t remaining = toRead - sramAddr;
        uint16_t chunk = (remaining > 128) ? 128 : remaining;
        uint16_t outLen = 0;
        uint8_t tmpBuf[140];
        if (!ds.readSequencer(sramAddr, tmpBuf, chunk, outLen)) {
            DBG_PRINTLN("SequencerBatch: readSequencer failed");
            return false;
        }
        // tmpBuf[0] = result byte (discard), tmpBuf[1..outLen-1] = sram data
        uint16_t dataBytes = (outLen > 1) ? outLen - 1 : 0;
        if (dataBytes > 0) {
            memcpy(resultBuf + destPos, tmpBuf + 1, dataBytes);
            destPos += dataBytes;
        }
        sramAddr += dataBytes;
    }

    resultLen = destPos;  // total bytes in resultBuf
    executed = true;

    if (result != 0xAA) {
        DBG_PRINT("SequencerBatch: runSequencer result=0x");
        if (DS28E18_Debug) Serial.println(result, HEX);
        DBG_PRINTLN("SRAM dump on failure:");
        for (uint16_t i = 1; i < resultLen; i++) {
             if (DS28E18_Debug) {
                 Serial.print(resultBuf[i], HEX);
                 Serial.print(" ");
             }
        }
        if (DS28E18_Debug) Serial.println();
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------
// getReadResult  —  copy data for a handle into caller's buffer
// -----------------------------------------------------------------------
bool SequencerBatch::getReadResult(int8_t handle, uint8_t *outBuf, uint16_t len) const {
    const uint8_t *ptr = getReadPtr(handle);
    if (!ptr) return false;

    uint16_t available = reads[handle].dataLen;
    uint16_t toCopy = (len < available) ? len : available;
    memcpy(outBuf, ptr, toCopy);
    return true;
}

// -----------------------------------------------------------------------
// getReadPtr  —  direct pointer into the readback buffer
// -----------------------------------------------------------------------
const uint8_t* SequencerBatch::getReadPtr(int8_t handle) const {
    if (!executed) return nullptr;
    if (handle < 0 || handle >= (int8_t)readCount) return nullptr;

    // The readSequencer response for the first 128 bytes returns:
    // [result_byte] [seq_byte_0] [seq_byte_1] ...
    // So sequencer SRAM offset N maps to resultBuf[N + 1].
    uint16_t bufOffset = reads[handle].sramOffset + 1;

    if (bufOffset + reads[handle].dataLen > resultLen) return nullptr;
    return &resultBuf[bufOffset];
}

// -----------------------------------------------------------------------
// getReadLen
// -----------------------------------------------------------------------
uint16_t SequencerBatch::getReadLen(int8_t handle) const {
    if (handle < 0 || handle >= (int8_t)readCount) return 0;
    return reads[handle].dataLen;
}
