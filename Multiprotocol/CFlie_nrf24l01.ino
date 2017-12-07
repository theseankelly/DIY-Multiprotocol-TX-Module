/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(CFLIE_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define CFLIE_PACKET_PERIOD 10000
#define CFLIE_PAYLOAD_SIZE 31

static void __attribute__((unused)) CFLIE_init(uint8_t bind)
{
    NRF24L01_Initialize();

    // CRC, radio on
    NRF24L01_SetTxRxMode(TX_EN);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP)); 
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x01);              // Auto Acknowledgement for data pipe 0
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);          // Enable data pipe 0
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 3);              // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x13);         // 3 retransmits, 500us delay

    NRF24L01_WriteReg(NRF24L01_05_RF_CH, 2);                 // Hard coded to channel 2
    NRF24L01_SetBitrate(NRF24L01_BR_2M);                     // Hard coded to 2M

    NRF24L01_SetPower();
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);             // Clear data ready, data sent, and retransmit

    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rx_tx_addr, 5);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, 5);

    // this sequence necessary for module from stock tx
    NRF24L01_ReadReg(NRF24L01_1D_FEATURE);
    NRF24L01_Activate(0x73);                          // Activate feature register
    NRF24L01_ReadReg(NRF24L01_1D_FEATURE);

    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x01);       // Enable Dynamic Payload Length on pipe 0
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x06);     // Enable Dynamic Payload Length, enable Payload with ACK
}

static void __attribute__((unused)) CFLIE_send_packet()
{
    struct CommanderPacketCppmEmu {
        struct {
            uint8_t numAuxChannels : 4;
            uint8_t reserved : 4;
        } hdr;
        uint16_t channelRoll;
        uint16_t channelPitch;
        uint16_t channelYaw;
        uint16_t channelThrust;
        uint16_t channelAux[10];
    } __attribute__((packed)) cpkt;

    cpkt.hdr.numAuxChannels = 10;

    // Remap RPYT
    cpkt.channelRoll = Servo_data[CH_AETR[0]];
    cpkt.channelPitch = Servo_data[CH_AETR[1]];
    cpkt.channelYaw = Servo_data[CH_AETR[3]];
    cpkt.channelThrust = Servo_data[CH_AETR[2]];

    // Aux channels
    for (uint8_t i = 4; i < 13; i++)
    {
        cpkt.channelAux[i - 4] = Servo_data[CH_AETR[i]];
    }

    // Construct the CRTP packet
    packet[0] = 0x70; // CRTP generic setpoint port and channel 
    packet[1] = 0x03; // CRTP CPPM Emulation packet type
    memcpy(&packet[2], (uint8_t*)&cpkt, sizeof(cpkt));

    NRF24L01_WriteReg(NRF24L01_07_STATUS, (_BV(NRF24L01_07_TX_DS) | _BV(NRF24L01_07_MAX_RT)));
    NRF24L01_FlushTx();
    NRF24L01_FlushRx();

    NRF24L01_WritePayload(packet, CFLIE_PAYLOAD_SIZE);

    // Keep Tx Power updated
    NRF24L01_SetPower();
}

uint16_t CFLIE_callback()
{
    CFLIE_send_packet();
    return CFLIE_PACKET_PERIOD;
}

uint16_t initCFLIE(void)
{
    bind_counter = 0;
    rx_tx_addr[0] =
    rx_tx_addr[1] =
    rx_tx_addr[2] =
    rx_tx_addr[3] =
    rx_tx_addr[4] = 0xE7; // CFlie uses fixed address

    CFLIE_init(IS_AUTOBIND_FLAG_on);
    BIND_DONE; // No binding support - hard coded rate, channel and addr

    return 50000;
}

#endif
