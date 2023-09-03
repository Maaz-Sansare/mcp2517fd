/*
 * This header file defined mcp2517fd class 
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

uint8_t spi_exchange(uint8_t send);
void spi_select_slave();
void spi_deselect_slave();
void debugLog(const char* log);
void delayMs(uint32_t delay_ms);

#define READ_BIT(reg, pos)      (((reg) & ((0x1UL) << (pos))) >> (pos))
#define SET_BIT(reg, pos)       ((reg) |= (0x1UL) << (pos))
#define CLEAR_BIT(reg, pos)     ((reg) &= ~ ((0x1UL) << (pos)))

// cmd
#define RESET_CMD (0b0000)
#define READ_CMD (0b0011)
#define READCRC_CMD (0b1011)
#define WRITE_CMD (0b0010)
#define WRITECRC_CMD (0b1010)

// Address
#define CONFIG_ADDRESS (0x000)
#define NBTCFG_ADDRESS (0x04)
#define IOCON_ADDRESS (0xE04)
#define OSC_ADDRESS (0xE00)
#define TXQCON_ADDRESS (0x50)
#define TXQSTA_ADDRESS (0x54)
#define FIFO_BASE_ADDRESS (0x5C)
#define FIFOSTATS_BASE_ADDRESS (0x60)
#define FIFOUA_BASE_ADDRESS (0x64)
#define INT_ADDRESS (0x01C)

#define RETRY_COUNT (0xFFFE - 1)
#define PLLRDY (1)
#define GPIO0 (0)
#define GPIO1 (1)
#define OUTPUT_MODE (0)
#define INPUT_MODE (1)
#define FIFO_OFFSET (0xC)
#define TX_FIFO (16 * FIFO_OFFSET)
#define RX_FIFO (1 * FIFO_OFFSET)
#define RAM_OFFSET (0x400)

typedef enum : uint8_t {
    enMCP2517FD_ErrorNoError = 0,
    enMCP2517FD_ErrorTimeout,
    enMCP2517FD_ErrorCRC,
    enMCP2517FD_ErrorRead,
    enMCP2517FD_ErrorWrte,
    enMCP2517FD_ErrorInternalError,
    enMCP2517FD_ErrorFpIsNull,
    enMCP2517FD_ErrorNullArg,
    enMCP2517FD_ErrorNotConfigured,
    enMCP2517FD_ErrorAlreadyConfigured,
    enMCP2517FD_ErrorInvalidArg,
    enMCP2517FD_ErrorBusy
} mcp2517fdError_t;

typedef enum : uint8_t {
    enMCP2517FD_ModeNormalFD = 0,
    enMCP2517FD_ModeSleep = 1,
    enMCP2517FD_ModeInternalLoopBack = 2,
    enMCP2517FD_ModeListenOnly = 3,
    enMCP2517FD_ModeConfiguration = 4,
    enMCP2517FD_ModeExternalLoopBack = 5,
    enMCP2517FD_ModeNormal20B = 6,
    enMCP2517FD_ModeRestrictedOperation = 7
} mcp2517fdMode_t ;

typedef enum : uint8_t {
    enCANFDFrameType_Remote = 0,
    enCANFDFrameType_Data = 1
} CANFDFrameType_t ;

typedef enum : uint8_t {
    enMCP2517FD_Size8Bytes = 8,
    enMCP2517FD_Size12Bytes = 12,
    enMCP2517FD_Size16Bytes = 16,
    enMCP2517FD_Size20Bytes = 20,
    enMCP2517FD_Size24Bytes = 24,
    enMCP2517FD_Size32Bytes = 32,
    enMCP2517FD_Size48Bytes = 48,
    enMCP2517FD_Size64Bytes = 64
} mcp2517fdSize_t;

typedef struct {
    bool isFrameStd;
    uint32_t identifier;
    CANFDFrameType_t frameType;
    mcp2517fdSize_t length;
    uint8_t* data;
} CANFDMessage_t;

// typedef struct {
//     uint32_t mask;
// } mcp2517fdFilterConfig_t;

typedef struct {
    bool crc;
    uint32_t baudrate;
    mcp2517fdSize_t payload;
    bool retransmission;
    mcp2517fdSize_t txFifo;
    mcp2517fdSize_t rxFifo;
} mcp2517fdConfig_t;

//* command(4bit), Address(12bit), Data(1 to n bytes), Number of bytes(1 bytes), Crc(2 bytes) */
class mcp2517fd {
    public:
        mcp2517fd(){};
        ~mcp2517fd(){};
        mcp2517fdError_t init(const mcp2517fdConfig_t* const config);
        mcp2517fdError_t setGpioMode(const int pin, const bool isInput);
        mcp2517fdError_t getGpioMode(const int pin, uint8_t* const mode);
        mcp2517fdError_t writeGpio(const int pin, const bool enable);
        mcp2517fdError_t readGpio(const int pin, bool* const state);
        bool isBusy();
        mcp2517fdError_t setMode(const mcp2517fdMode_t mode);
        mcp2517fdError_t getMode(mcp2517fdMode_t* mode);
        mcp2517fdError_t setFilter(const uint32_t mask);
        mcp2517fdError_t write(CANFDMessage_t* const message);
        mcp2517fdError_t read(CANFDMessage_t* const message);
        bool hasMsg();

    private:
        //* Need to add support for timeout for non-blocking spi...
        mcp2517fdError_t mcp2517fd_read(uint16_t address, uint8_t* readDataBuffer, size_t size, bool checkCrc) {
            spi_select_slave();
            if (checkCrc) {
                spi_exchange((READCRC_CMD << 4) | (address >> 8));
            } else {
                spi_exchange((READ_CMD << 4) | (address >> 8));
            }
            spi_exchange(address);
            if (checkCrc) {
                spi_exchange(size);
            }

            for (size_t i = 0; i < size; i++) {
                readDataBuffer[i] = spi_exchange(0xFF);
            }

            if (checkCrc) {
                // TODO needs to add a crc check..
                uint16_t crc = 0;
                crc = spi_exchange(0xFF) << 8;
                crc |= spi_exchange(0xFF);
            }

            spi_deselect_slave();
            return enMCP2517FD_ErrorNoError;
        }

        mcp2517fdError_t mcp2517fd_write(uint16_t address, uint8_t* writeDataBuffer, size_t size, bool checkCrc) {
            spi_select_slave();
            if (checkCrc) {
                spi_exchange((WRITECRC_CMD << 4) | (address >> 8));
            } else {
                spi_exchange((WRITE_CMD << 4) | (address >> 8));
            }
            spi_exchange(address);
            if (checkCrc) {
                spi_exchange(size);
            }

            for (size_t i = 0; i < size; i++) {
                spi_exchange(writeDataBuffer[i]);
            }

            if (checkCrc) {
                // TODO needs to add a crc check..
                uint16_t crc = 0;
                spi_exchange(crc >> 8);
                spi_exchange(crc);
            }

            spi_deselect_slave();
            return enMCP2517FD_ErrorNoError;
        }

        mcp2517fdError_t mcp2517fd_writeMessage(CANFDMessage_t* message, bool checkCrc) {
            if (!isMCP2517FDConfigured) {
                //if (debugLog) {
                    debugLog("Not configured.\n");
                //}
                return enMCP2517FD_ErrorNotConfigured;
            }
            // Read fifo stats
            uint8_t stats = 0;
            mcp2517fd_read(TXQSTA_ADDRESS, &stats, 1, false);
            if (((stats >> 4) & 1)) {
                return enMCP2517FD_ErrorBusy; // Interrupt pending..
            }
            /*mcp2517fd_read((FIFOSTATS_BASE_ADDRESS + TX_FIFO), &stats, 1, false);
            if ((stats >> 2) & 1) {
                return enMCP2517FD_ErrorBusy; // Interrupt pending..
            }*/
            // Read Fifo write address
            uint16_t fifoAddress = 0;
            mcp2517fd_read((FIFOUA_BASE_ADDRESS + TX_FIFO),(uint8_t*)&fifoAddress, 2, false);
            fifoAddress += RAM_OFFSET;

            spi_select_slave();
            if (checkCrc) {
                spi_exchange((WRITECRC_CMD << 4) | (fifoAddress >> 8));
            } else {
                spi_exchange((WRITE_CMD << 4) | (fifoAddress >> 8));
            }
            spi_exchange(fifoAddress);
            if (checkCrc) {
                //spi_exchange(size);
            }

            uint8_t temp = 0;
            for (size_t i = 0; i < 4; i++) {
                // Sending identifier
                temp = message->identifier >> (i * 8);
                spi_exchange(temp);
            }

            switch (message->length) {
                case enMCP2517FD_Size8Bytes :
                    temp = 9;
                    break;
                case enMCP2517FD_Size12Bytes :
                    temp = 9;
                    break;
                case enMCP2517FD_Size16Bytes :
                    temp = 10;
                    break;
                case enMCP2517FD_Size20Bytes :
                    temp = 11;
                    break;
                case enMCP2517FD_Size24Bytes :
                    temp = 12;
                    break;
                case enMCP2517FD_Size32Bytes :
                    temp = 13;
                    break;
                case enMCP2517FD_Size48Bytes :
                    temp = 14;
                    break;
                case enMCP2517FD_Size64Bytes :
                    temp = 15;
                    break;
            }
            //temp |= message->isFrameStd ? ~(1 << 4) : 1 << 4;
            //temp |= (message->frameType == enCANFDFrameType_Remote) ? (1 << 5) : ~(1 << 5);
            spi_exchange(temp);
            for (size_t i = 0; i < (3 + 4); i++) {
                spi_exchange(0);
            }

            for (size_t i = 0; i < message->length; i++) {
                spi_exchange(message->data[i]);
            }

            if (checkCrc) {
                // TODO needs to add a crc check..
                uint16_t crc = 0;
                spi_exchange(crc >> 8);
                spi_exchange(crc);
            }
            spi_deselect_slave();

            // Increment FIFO, send message (see DS20005688B, page 48)
            temp = (1 << 0) | (1 << 1) ; // Set UINC bit, TXREQ bit
            mcp2517fd_write(TXQCON_ADDRESS + 1, &temp, 1, false);

            return enMCP2517FD_ErrorNoError;
        }

        mcp2517fdError_t mcp2517fd_readMessage(CANFDMessage_t* message, bool checkCrc) {
            if (!isMCP2517FDConfigured) {
                //if (debugLog) {
                    debugLog("Not configured.\n");
                //}
                return enMCP2517FD_ErrorNotConfigured;
            }
            // Read fifo stats
            uint8_t stats = 0;
            mcp2517fd_read((FIFOSTATS_BASE_ADDRESS + RX_FIFO), &stats, 1, false);
            if (!(stats & 1)) {
                return enMCP2517FD_ErrorBusy; // Interrupt pending..
            }
            // Read Fifo write address
            uint16_t fifoAddress = 0;
            mcp2517fd_read((FIFOUA_BASE_ADDRESS + RX_FIFO),(uint8_t*)&fifoAddress, 2, false);
            fifoAddress += RAM_OFFSET;

            spi_select_slave();
            if (checkCrc) {
                spi_exchange((READCRC_CMD << 4) | (fifoAddress >> 8));
            } else {
                spi_exchange((READ_CMD << 4) | (fifoAddress >> 8));
            }
            spi_exchange(fifoAddress);

            message->identifier = 0;
            for (size_t i = 0; i < 4; i++) {
                // Getting identifier
                message->identifier |= (spi_exchange(0xFF) << (i * 8));
            }

            uint8_t temp = spi_exchange(0xFF);
            //message->length = (temp & 0xF) - 8;
            switch ((temp & 0xF)) {
                case 9 :
                    message->length = enMCP2517FD_Size12Bytes;
                    break;
                case 10 :
                    message->length = enMCP2517FD_Size16Bytes;
                    break;
                case 11 :
                    message->length = enMCP2517FD_Size20Bytes;
                    break;
                case 12 :
                    message->length = enMCP2517FD_Size24Bytes;
                    break;
                case 13 :
                    message->length = enMCP2517FD_Size32Bytes;
                    break;
                case 14 :
                    message->length = enMCP2517FD_Size48Bytes;
                    break;
                case 15 :
                    message->length = enMCP2517FD_Size64Bytes;
                    break;
                case 0 :
                    return enMCP2517FD_ErrorBusy;
            }
            message->isFrameStd = (temp & 1 >> 4) ? false : true;
            message->frameType = (temp & 1 >> 5) ? enCANFDFrameType_Remote : enCANFDFrameType_Data;

            for (size_t i = 0; i < (3 + 4); i++) {
                spi_exchange(0xFF);
            }

            for (size_t i = 0; i < message->length; i++) {
                message->data[i] = spi_exchange(0xFF);
            }

            if (checkCrc) {
                // TODO needs to add a crc check..
                uint16_t crc = 0;
                spi_exchange(crc >> 8);
                spi_exchange(crc);
            }
            spi_deselect_slave();

            // Increment FIFO, send message (see DS20005688B, page 48)
            //temp = (1 << 0) | (1 << 1) ; // Set UINC bit, TXREQ bit
            //mcp2517fd_write(TXQCON_ADDRESS + 1, &temp, 1, false);

            return enMCP2517FD_ErrorNoError;
        }

        mcp2517fdError_t mcp2517fd_reset() {
            spi_select_slave();
            spi_exchange((RESET_CMD << 4) | (CONFIG_ADDRESS >> 8));
            spi_exchange(CONFIG_ADDRESS);
            spi_deselect_slave();
            return enMCP2517FD_ErrorNoError;
        }

        bool isMCP2517FDConfigured = false;
        bool isCrcEnable = false;
};