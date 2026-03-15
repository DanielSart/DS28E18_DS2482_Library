#ifndef SEQUENCER_BATCH_H
#define SEQUENCER_BATCH_H

#include <Arduino.h>
#include "DS28E18.h"
#include "DS28E18_Sequencer.h"

// Maximum number of read operations that can be tracked in one batch.
// Each addI2CRead() or addI2CWriteRead() consumes one slot.
#define BATCH_MAX_READS 16

// -----------------------------------------------------------------------
// SequencerBatch
//
// Accumulates multiple I2C operations into a single DS28E18 sequencer
// buffer, tracks where each read-response will land in SRAM, then
// executes everything in one 1-Wire round-trip and lets the caller
// extract individual results by handle.
//
// Usage:
//   SequencerBatch batch;
//   batch.clear();
//   int8_t h0 = batch.addI2CWriteRead(0x44, cmd, 1, 6, 20);
//   int8_t h1 = batch.addI2CWriteRead(0x44, cmd, 1, 6, 20);
//   batch.execute(ds28e18);
//   batch.getReadResult(h0, buf0, 6);
//   batch.getReadResult(h1, buf1, 6);
// -----------------------------------------------------------------------
class SequencerBatch {
public:
    SequencerBatch();

    // Reset the batch for new use
    void clear();

    // ---- Building the batch ----

    // Add a write-only I2C transaction: START + WRITE(addr, data) + STOP
    bool addI2CWrite(uint8_t i2cAddr, const uint8_t *data, uint8_t dataLen);

    // Add a read-only I2C transaction: START + READ(addr, readLen) + STOP
    // Returns a handle (>=0) to retrieve the read data later, or -1 on error.
    int8_t addI2CRead(uint8_t i2cAddr, uint16_t readLen);

    // Add a combined write-then-read I2C transaction:
    //   START + WRITE(addr, data) + STOP + DELAY + START + READ(addr, readLen) + STOP
    // The delayMs parameter specifies how long to wait between write and read.
    // Returns a handle (>=0) to retrieve the read data later, or -1 on error.
    int8_t addI2CWriteRead(uint8_t i2cAddr,
                           const uint8_t *writeData, uint8_t writeLen,
                           uint16_t readLen,
                           uint16_t delayMs = 0);

    // Add a raw delay to the sequence.
    // delayMs: delay in milliseconds (1-256 ms, mapped to DS28E18 delay value)
    bool addDelay(uint16_t delayMs);

    // ---- Execution ----

    // Write the accumulated sequence to SRAM, run it, and read back results.
    // tOP_extra_ms: additional execution delay on top of the accumulated delays.
    bool execute(DS28E18 &ds, uint16_t tOP_extra_ms = 5);

    // ---- Result extraction ----

    // Copy the read data for a given handle into outBuf.
    // Returns true if the handle is valid and data was copied.
    bool getReadResult(int8_t handle, uint8_t *outBuf, uint16_t len) const;

    // Get a pointer directly into the result buffer for a given handle.
    // Returns nullptr if handle is invalid.
    const uint8_t* getReadPtr(int8_t handle) const;

    // Get the length (number of data bytes) for a given handle.
    uint16_t getReadLen(int8_t handle) const;

    // ---- Status ----

    // Number of read handles allocated so far
    uint8_t numReads() const { return readCount; }

    // Remaining capacity in the sequencer buffer (bytes)
    uint16_t remainingCapacity() const { return seq.remainingCapacity(); }

    // Total actual accumulated delay in the batch (ms) after quantization to
    // DS28E18 delay steps — useful for tOP estimation.
    uint16_t totalDelayMs() const { return accumDelayMs; }

    // Was the batch successfully executed?
    bool wasExecuted() const { return executed; }

private:
    DS28E18_Sequencer seq;

    // Track where each read's data lands in the readback buffer.
    // After runSequencer, the SRAM is read back: byte 0 is result code,
    // then the sequencer bytes follow with read placeholders filled in.
    struct ReadSlot {
        uint16_t sramOffset;   // offset in SRAM where data bytes start
        uint16_t dataLen;      // number of data bytes
    };

    ReadSlot reads[BATCH_MAX_READS];
    uint8_t readCount;

    // Running SRAM offset tracker.
    // After readSequencer, response is: [result_byte] [sequencer_bytes...]
    // So sequencer byte 0 maps to readback byte 1.
    // We track the sequencer-internal offset (0-based), then add 1 at extraction time.
    uint16_t sramCursor;

    // Accumulated delay for tOP estimation
    uint16_t accumDelayMs;

    bool executed;

    // Raw readback buffer
    uint8_t resultBuf[SEQ_MAX_SIZE];
    uint16_t resultLen;

    // Helper: convert requested milliseconds to DS28E18 delay encoding.
    static uint8_t msToDelayVal(uint16_t ms);
};

#endif
