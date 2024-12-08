#include "can.h"
#include "uart.h"

#define SEL_FILTER_BANK             0

void on_std_msg_received() {
    uint8_t data[8] = {0};

    // The ID is standard, extract the ID
    uint16_t id = (CAN_RI0R_STID & CAN1->sFIFOMailBox[0].RIR) >> CAN_TI0R_STID_Pos;

    // Get length of the data (DLC - Data Length Code)
    uint8_t dlc = ((CAN1->sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC) >> CAN_RDT0R_DLC_Pos);

    // copy the data
    data[0] = (uint8_t)((CAN_RDL0R_DATA0 & CAN1->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA0_Pos);
    data[1] = (uint8_t)((CAN_RDL0R_DATA1 & CAN1->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA1_Pos);
    data[2] = (uint8_t)((CAN_RDL0R_DATA2 & CAN1->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA2_Pos);
    data[3] = (uint8_t)((CAN_RDL0R_DATA3 & CAN1->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA3_Pos);
    data[4] = (uint8_t)((CAN_RDH0R_DATA4 & CAN1->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA4_Pos);
    data[5] = (uint8_t)((CAN_RDH0R_DATA5 & CAN1->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA5_Pos);
    data[6] = (uint8_t)((CAN_RDH0R_DATA6 & CAN1->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA6_Pos);
    data[7] = (uint8_t)((CAN_RDH0R_DATA7 & CAN1->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA7_Pos);

    // Release the RX FIFO 0
    CAN1->RF0R |= CAN_RF0R_RFOM0;

    uart1_send_string("Msg(id:%X, len: %d) data[0]: %u, data[1]: %u", id, dlc, data[0], data[1]);
}

void handle_can_message() {

    if(0 != (CAN1->IER & CAN_IER_FMPIE0)) {
        // FIFO-0 message pending interrupt has enabled

        if(CAN1->RF0R & CAN_RF0R_FMP0) {
            // There is a pending message in the FIFO

            // Get the identifier extension bit and check
            // if the ID is standard or extended mode.
            if(0 == (CAN1->sFIFOMailBox[0].RIR & CAN_RI0R_IDE)) {
                on_std_msg_received();
            }

        }
    }
}

 void USB_LP_CAN1_RX0_IRQHandler(void) {
    handle_can_message();
 }

void CAN1_RX1_IRQHandler(void) {
    handle_can_message();
}

void can_init() {
    uint32_t prio_grp;

    // Enable the clock for CAN peripheral
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

    // CAN pins are PA11 and PA12, enable GIPOA clock
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // Configure PA11(CAN RX) as input floating
    // This is reset state, no need to change register value

    // Configure PA12 (CAN TX) as output alternate function push pull
    GPIOA->CRH |= GPIO_CRH_CNF12_1 | GPIO_CRH_MODE12_0;

    // Enable CAN interrupt
    prio_grp = NVIC_GetPriorityGrouping();
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, NVIC_EncodePriority(prio_grp, 0, 0));
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    NVIC_SetPriority(CAN1_RX1_IRQn, NVIC_EncodePriority(prio_grp, 0, 0));
    NVIC_EnableIRQ(CAN1_RX1_IRQn);

    // Request CAN initialization and wait until 
    // the CAN complete initialization
    CAN1->MCR |= CAN_MCR_INRQ;
    while(0 == (CAN1->MSR & CAN_MSR_INAK));

    // Exit from sleep mode, and wait until the CAN exit
    // from the sleep mode
    CAN1->MCR &= ~CAN_MCR_SLEEP;
    while(0 != (CAN1->MSR & CAN_MSR_SLAK));

    // Leaving the rest of configurations in
    // the MCR as disabled

    // Set bit timing register. Loopback, sync jump
    // width as 0, TS1 = 2TQ, and TS2 = 1TQ
    CAN1->BTR |= (uint32_t) (CAN_BTR_LBKM | CAN_BTR_TS1_0);
}

void can_config_filter() {
    const uint32_t message_id = MESSAGE_ID << CAN_F0R1_FB21_Pos;
    const uint32_t filter_mask = FILTER_MASK << CAN_F0R1_FB21_Pos;

    // FINIT bit has to be set to initialize CAN filter bank
    // related registers
    CAN1->FMR |= CAN_FMR_FINIT;

    // We are using the filter bank SEL_FILTER_BANK to filter the
    // received packets. Inorder to configure filter bank 0,
    // deactivate the filter bank first.
    CAN1->FA1R &= ~(1 << SEL_FILTER_BANK);

    // Configure filter scale as single 32 bit for SEL_FILTER_BANK
    CAN1->FS1R |= (1 << SEL_FILTER_BANK);

    CAN1->sFilterRegister[SEL_FILTER_BANK].FR1 = message_id;
    CAN1->sFilterRegister[SEL_FILTER_BANK].FR2 = filter_mask;

    // Configure filter mode to use FR1 as ID and FR2 as Mask
    CAN1->FM1R &= ~(1 << SEL_FILTER_BANK);

    // Assign FIFO-0 for SEL_FILTER_BANK
    CAN1->FFA1R &= ~(1 << SEL_FILTER_BANK);

    // Finally re-enable filter bank 0
    CAN1->FA1R |= (1 << SEL_FILTER_BANK);

    // Leave from the initialization mode
    CAN1->FMR &= ~CAN_FMR_FINIT;
}

void can_start() {
    // Request to leave from initialization state
    // and wait until it's acknowleged.
    CAN1->MCR &= ~CAN_MCR_INRQ;
    while(0 != (CAN1->MSR & CAN_MSR_INAK));

    // Enable the FIFO-0 message pending interrupt
    CAN1->IER |= CAN_IER_FMPIE0;
}

void can_send_message(uint8_t *data, uint8_t len, uint16_t std_id) {
    uint8_t tx_mailbox;

    if(0 != (CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2))) {
        // Atleast one transmit mail box is not empty

        // Get the mail box to be used for sending data
        tx_mailbox = (CAN1->TSR & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;

        // Set the standard message ID, Set RTR is zero
        // since we are sending the data rather than requesting.
        CAN1->sTxMailBox[tx_mailbox].TIR = (std_id << CAN_TI0R_STID_Pos);

        // Set DLC - data length code
        CAN1->sTxMailBox[tx_mailbox].TDTR = len;

        // Set up the data field
        CAN1->sTxMailBox[tx_mailbox].TDLR = ((data[1] << CAN_TDL0R_DATA1_Pos) |
                                             (data[0] << CAN_TDL0R_DATA0_Pos));

        // Request data transmission
        CAN1->sTxMailBox[tx_mailbox].TIR |= CAN_TI0R_TXRQ;
    }
}
