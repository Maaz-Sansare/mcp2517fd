
#include "mcp2517fd.hpp"

mcp2517fdError_t mcp2517fd::init(const mcp2517fdConfig_t* const config) {
    //if ((!spi_exchange) || (!spi_select_slave) || (!spi_deselect_slave) || !(delayMs)) {
        //if (debugLog) {
            debugLog("Function pointer in null.\n");
        //
        //}
        //return enMCP2517FD_ErrorFpIsNull;
    //}
    if (config == NULL) {
        //if (debugLog) {
            debugLog("Null configuration passed.\n");
        //}
        return enMCP2517FD_ErrorNullArg;
    }
    if (isMCP2517FDConfigured) {
        //if (debugLog) {
            debugLog("Already configured.\n");
        //}
        return enMCP2517FD_ErrorAlreadyConfigured;
    }

    // Set MCP2517FD in configuration mode..
    mcp2517fdError_t error = setMode(enMCP2517FD_ModeConfiguration);
    if (error != enMCP2517FD_ErrorNoError) {
        return error;
    }
    //if (debugLog) {
        debugLog("Device is in cofiguration mode.\n");
    //}
    mcp2517fd_reset();
    //if (debugLog) {
        debugLog("Reset done.\n");
    //}

    uint8_t temp[255];
    uint8_t read[255];
    memset(temp,0xAA,sizeof(temp));
    memset(read,0,sizeof(read));
    for (uint16_t offset = 0 ; offset < 0x800 ; offset += 255) {
        mcp2517fd_write(RAM_OFFSET + offset, temp, sizeof(temp), false) ;
        mcp2517fd_read(RAM_OFFSET + offset, read, sizeof(read), false) ;
        if (memcmp(temp, read, sizeof(temp)) != 0) {
            //if (debugLog) {
                debugLog("RAM test failed.\n");
            //}
        }
    }
    //if (debugLog) {
        debugLog("RAM reset done.\n");
    //}
    // mcp2517fd is in configuration mode ...
    mcp2517fdMode_t mode;
    getMode(&mode);
    if (mode != enMCP2517FD_ModeConfiguration) 
        while(1);
    // do clock setup
    uint8_t setup = 0; // PLL bit : 1 to Enable 0 to diable..
    //! Enable PLL is not working...
    uint16_t retry = 0;
    mcp2517fd_write(OSC_ADDRESS, &setup, 1, false); // Enable PLL // TODO clock setup based on user input
    // wait till pll is ready 
    setup = 0;
    do {
        mcp2517fd_read(OSC_ADDRESS + 0x1, &setup, 1, false);
        /*if ((setup & PLLRDY) == 1) {
            //if (debugLog) {
                debugLog("PLL is ready.\n");
            //}
            break;
        }*/
        if ((setup >> 2) == 1) {
            //if (debugLog) {
                debugLog("Clock is stable and ready.\n");
            //}
            break;
        }
    } while (retry++ <= RETRY_COUNT);
    if (retry >= RETRY_COUNT) {
        /*//if (debugLog) {
            debugLog("PLL timeout.\n");
        }*/
        //if (debugLog) {
            debugLog("Clock is not stable and ready.\n");
        //}
        return enMCP2517FD_ErrorTimeout;
    }

    // Reset RAM
    memset(temp,0,sizeof(temp));
    for (uint16_t offset = 0 ; offset < 0x800 ; offset += 255) {
        mcp2517fd_write(0x400 + offset, temp, sizeof(temp), false) ;
    }
    //if (debugLog) {
        debugLog("RAM reset done.\n");
    //}

    // Configure ISO CRC Enable bit
    setup = 1 << 6 ; // PXEDIS
    if (config->crc) {
        setup |= 1 << 5 ; // Enable ISO CRC in CAN FD Frames bit
    }
    mcp2517fd_write(CONFIG_ADDRESS, &setup, 1, false);

    // Configure TXQ and TXAT
    setup = config->retransmission ? 2 << 6 : ~(3 << 6);
    mcp2517fd_write(TXQCON_ADDRESS + 0x2, &setup, 1, false);
    setup = config->payload << 5 | 0x1F << 0; //? Fifo message deep...
    mcp2517fd_write(TXQCON_ADDRESS + 0x3, &setup, 1, false);
    setup = config->retransmission ? 1 : 0;
    setup |= 1 << 4;
    mcp2517fd_write(CONFIG_ADDRESS + 0x2, &setup, 1, false);

    // Set FIFO size based on user input...
    // TODO There are total 32 fifo handler 
    //      lets 24 fifo be configured as RX fifo and remaining will be tx
    // Configure RX FIFO
    setup = 1 << 2;
    mcp2517fd_write(TXQCON_ADDRESS + 1, &setup, 1, false);
    setup = config->rxFifo << 5 | 0x1F << 0;
    mcp2517fd_write(FIFO_BASE_ADDRESS + RX_FIFO + 0x3, &setup, 1, false);
    setup = 1 << 3 | 1 << 2 | 1 << 0; // TFNRFNIE : rxFifo not empty , TFNRFFIE : rxFifo full , RXOVIE : overflow enable
    mcp2517fd_write(FIFO_BASE_ADDRESS + RX_FIFO, &setup, 1, false);
    // Configure TX FIFO
    setup = config->txFifo << 5 | 0x1F << 0;
    mcp2517fd_write(FIFO_BASE_ADDRESS + TX_FIFO + 0x3, &setup, 1, false);
    /*setup = 1 << 4; // TXATIE
    mcp2517fd_write(FIFO_BASE_ADDRESS + TX_FIFO, &setup, 1, false);*/
    setup = 1 << 7 | 1 << 4; // TXEN : Tx fifo , TXATIE : enable interrupt
    mcp2517fd_write(FIFO_BASE_ADDRESS + TX_FIFO, &setup, 1, false);
    //if (debugLog) {
        debugLog("Fifo config done.\n");
    //}

    // Configure receive filters
    // set allow all filter user can set by calling setFilter api...

    // Activate interrupts
    setup = 1 << 1 | 1 << 0; // TXIF : Tx interrupt Enable , RXIF : Rx interrupt Enable
    mcp2517fd_write(INT_ADDRESS + 0x2, &setup, 1, false);
    setup = 1 << 2; // TXATIE : Transmit Attempt Interrupt Enable bit
    mcp2517fd_write(INT_ADDRESS + 0x3, &setup, 1, false);
    //if (debugLog) {
        debugLog("Enable interrupts.\n");
    //}

    // Program nominal bit rate
    // mcp2517fd_write(NBTCFG_ADDRESS, &setup, 1, false);

    // Program data bit rate

    isMCP2517FDConfigured = true;
    //if (debugLog) {
        debugLog("Configuration done.\n");
    //}
    return enMCP2517FD_ErrorNoError;
}

mcp2517fdError_t mcp2517fd::setGpioMode(const int pin, const bool isInput) {
    if (!isMCP2517FDConfigured) {
        //if (debugLog) {
            debugLog("Not configured.\n");
        //}
        return enMCP2517FD_ErrorNotConfigured;
    }
    if (pin <= GPIO1) {
        uint8_t mode = 0;
        mcp2517fd_read(IOCON_ADDRESS, &mode, 1, false);
        if (isInput) {
            SET_BIT(mode, pin);
        } else {
            CLEAR_BIT(mode, pin);
        }
        mcp2517fd_write(IOCON_ADDRESS, &mode, 1, false);
        return enMCP2517FD_ErrorNoError;
    } else {
        return enMCP2517FD_ErrorInvalidArg;
    }
}

mcp2517fdError_t mcp2517fd::getGpioMode(const int pin, uint8_t* const mode) {
    if (!isMCP2517FDConfigured) {
        //if (debugLog) {
            debugLog("Not configured.\n");
        //}
        return enMCP2517FD_ErrorNotConfigured;
    }
    if (pin <= GPIO1) {
        uint8_t gmode = 0;
        mcp2517fd_read(IOCON_ADDRESS, &gmode, 1, false);
        if (READ_BIT(gmode, pin)) {
            *mode = INPUT_MODE;
        } else {
            *mode = OUTPUT_MODE;
        }
        return enMCP2517FD_ErrorNoError;
    } else {
        return enMCP2517FD_ErrorInvalidArg;
    }
}

mcp2517fdError_t mcp2517fd::writeGpio(const int pin, const bool enable) {
    if (!isMCP2517FDConfigured) {
        //if (debugLog) {
            debugLog("Not configured.\n");
        //}
        return enMCP2517FD_ErrorNotConfigured;
    }
    if (pin <= GPIO1) {
        uint8_t state = 0;
        mcp2517fd_read(IOCON_ADDRESS + 0x1, &state, 1, false);
        if (enable) {
            SET_BIT(state, pin);
        } else {
            CLEAR_BIT(state, pin);
        }
        mcp2517fd_write(IOCON_ADDRESS + 0x1, &state, 1, false);
        return enMCP2517FD_ErrorNoError;
    } else {
        return enMCP2517FD_ErrorInvalidArg;
    }
}

mcp2517fdError_t mcp2517fd::readGpio(const int pin, bool* const state) {
    if (!isMCP2517FDConfigured) {
        //if (debugLog) {
            debugLog("Not configured.\n");
        //}
        return enMCP2517FD_ErrorNotConfigured;
    }
    if (pin <= GPIO1) {
        uint8_t gstate = 0;
        mcp2517fd_read(IOCON_ADDRESS + 0x2, &gstate, 1, false);
        if (READ_BIT(gstate, pin)) {
            *state = true;
        } else {
            *state = false;
        }
        return enMCP2517FD_ErrorNoError;
    } else {
        return enMCP2517FD_ErrorInvalidArg;
    }
}

bool mcp2517fd::isBusy() {
    if (!isMCP2517FDConfigured) {
        //if (debugLog) {
            debugLog("Not configured.\n");
        //}
        return true;
    }
    uint8_t isbusy = 0;
    mcp2517fd_read(IOCON_ADDRESS + 0x1, &isbusy, 1, false);

    return (READ_BIT(isbusy, 3)) ? true : false;
}

mcp2517fdError_t mcp2517fd::setMode(const mcp2517fdMode_t mode) {
    // Set MCP2517FD in user specified mode..
    uint8_t config_set_value = (mode | 1 << 3);
    uint8_t config_bit = 0;
    uint16_t retry = 0;
    mcp2517fd_write(CONFIG_ADDRESS + 0x3, &config_set_value, 1, false); // Abort all transmission and set to user specific mode..
    do {
        mcp2517fd_read(CONFIG_ADDRESS + 0x2, &config_bit, 1, false); // checking for config mode bit
        if ((config_bit >> 5) == mode) {
            break;
        }
    } while (retry++ != RETRY_COUNT);
    if (retry >= RETRY_COUNT) {
        //if (debugLog) {
            debugLog("Timeout error.\n");
        //}
        return enMCP2517FD_ErrorTimeout;
    }

    return enMCP2517FD_ErrorNoError;
}

mcp2517fdError_t mcp2517fd::getMode(mcp2517fdMode_t* mode) {
    // Get MCP2517FD in user specified mode..
    uint8_t config_bit = 0;
    mcp2517fd_read(CONFIG_ADDRESS + 0x2, &config_bit, 1, false); // checking for config mode bit
    *mode = (mcp2517fdMode_t)(config_bit >> 5);
    return enMCP2517FD_ErrorNoError;
}

mcp2517fdError_t mcp2517fd::write(CANFDMessage_t* const message) {
    if (message == NULL) {
        //if (debugLog) {
            debugLog("Null configuration passed.\n");
        //}
        return enMCP2517FD_ErrorNullArg;
    }
    return mcp2517fd_writeMessage(message, false);
}

mcp2517fdError_t mcp2517fd::read(CANFDMessage_t* const message) {
    if (message == NULL) {
        //if (debugLog) {
            debugLog("Null configuration passed.\n");
        //}
        return enMCP2517FD_ErrorNullArg;
    }
    return mcp2517fd_readMessage(message, false);
}