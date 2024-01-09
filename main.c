//*****************************************************************************
// Interview Demo
// Cameron Crowell
// 1/9/2023
// This code is in the domain of Cameron. Violators will be severely scolded.
//*****************************************************************************
#include <msp430.h>

//*****************************************************************************
// Pin Out
//*****************************************************************************
// Both MCUs
#define CONTROL_PIN     BIT0
#define TXD             BIT2
#define RXD             BIT1

// Primary MCU
#define BUTTON_1        BIT4
#define BUTTON_2        BIT5

// Secondary MCU
#define DIGIT_1s        BIT6
#define DIGIT_10s       BIT5
#define DIGIT_100s      BIT4
#define DIGIT_1000s     BIT3

//*****************************************************************************
// Variables
//*****************************************************************************
// Machine states
enum my_state_t
{
    AMI = 0,
    MSG,
    Presses
};

// Initialize in AMI state
volatile enum my_state_t state = AMI;

const unsigned int D0_parity = 0xA0;  // Parity bit is transmitted to track digits
const unsigned int D1_parity = 0xB0;
const unsigned int D2_parity = 0xC0;
const unsigned int D3_parity = 0xD0;
const unsigned int NULL_VAL = 15;     // Placement of Null value in the seven seg lookup table

unsigned int controllerFlag;          // Flag to ID MCU
volatile unsigned int timerCount = 0; // Tracks half seconds
volatile unsigned int buttonPressed = 0;

volatile unsigned int D0;             // Variables to store UART transmissions
volatile unsigned int D1;
volatile unsigned int D2;
volatile unsigned int D3;

//*****************************************************************************
// Function Prototypes
//*****************************************************************************
void enableGPIOBoard1(void);
void enableUARTBoard1(void);
void enableTimerA0(void);
void enablePort_1(void);

void UART_sendData(char);
void UART_encode(int, int);

void enableGPIOBoard2(void);
void enableUARTBoard2(void);

void smallDelay(void);
void sevenSeg(int);
void quadDigitArray(int, int, int, int);
void digitSeparator(int);

//*****************************************************************************
// Main
//*****************************************************************************
int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;            // Stop WDT on both chips
    DCOCTL = 0;                          // Select lowest DCOx and MODx settings<
    BCSCTL1 = CALBC1_1MHZ;               // Set DCO
    DCOCTL = CALDCO_1MHZ;

    controllerFlag = P1IN & CONTROL_PIN; // Determine if Primary or Secondary MCU

    // Board 1 Code
    if(controllerFlag == 0x01)
    {
        enableGPIOBoard1();
        enableUARTBoard1();
        enableTimerA0();
        enablePort_1();
        __bis_SR_register(GIE); // interrupts enabled

        while(1)
        {
            switch(state)
            {
                case AMI:
                    UART_encode(0, 12);         // I
                    UART_encode(1, 11);         // M
                    UART_encode(2, 10);         // A
                    UART_encode(3, NULL_VAL);   // Null
                    break;
                case MSG:
                    UART_encode(0, 14);         // G
                    UART_encode(1, 13);         // S
                    UART_encode(2, 11);         // M
                    UART_encode(3, NULL_VAL);   // Null
                    break;
                case Presses:
                    digitSeparator(buttonPressed); // Seperate the digits of buttonPressed to be transmitted
                    break;
            }
            UART_sendData(D0);
            UART_sendData(D1);
            UART_sendData(D2);
            UART_sendData(D3);
        }
    }

    // Board 2 Code
    if(controllerFlag == 0x00)
    {
        enableGPIOBoard2();
        enableUARTBoard2();
        __bis_SR_register(GIE); // interrupts enabled
        while(1)
        {
            quadDigitArray(D0, D1, D2, D3);
        }
    }
}

//**************************************************************************************************************************
// Setup Board 1
//**************************************************************************************************************************
//*****************************************************************************
// void enableGPIO1()
// Input: Void
// Output: void
// Functions: Enables GPIO Pins for MCU1
//*****************************************************************************
void enableGPIOBoard1(void)
{
    // Setting the UART function for P1.1 & P1.2
    P1SEL |= TXD; // P1.1 = RXD, P1.2=TXD
    P1SEL2 |= TXD; // P1.1 = RXD, P1.2=TXD
}

//*****************************************************************************
// void enableUARTBoard1()
// Input: Void
// Output: void
// Functions: Enables UART communications for MCU1
//*****************************************************************************
void enableUARTBoard1(void)
{
    UCA0CTL1 |= UCSSEL_2; // SMCLK
    UCA0BR0 = 104; // 1MHz 9600
    UCA0BR1 = 0; // 1MHz 9600
    UCA0MCTL = UCBRS1 + UCBRS0; // Modulation UCBRSx = 1
    UCA0CTL1 &= ~UCSWRST; // **Initialize USCI state machine**
}

//*****************************************************************************
// void enableTimerA0()
// Input: Void
// Output: void
// Functions: Enables Timer A0
//*****************************************************************************
void enableTimerA0(void)
{
    // Stop timer before adjusting configuration
    TA0CTL = MC_0;
    TA0CCTL0 |= CCIE;                   // CCR0 interrupt enabled
    TA0CTL |= TASSEL_2 + MC_1 + ID_3;   // SMCLK, upmode, /8
    TA0CCR0 = 62500;                    // 1,000,000 / 8 = 125,000 --> 1 sec timer /2 --> .5sec timer
}

//*************************************************************
// void enablePort_1(void)
// Input: Void
// Output: void
// Functions: Initialize Port 1 interrupt
//*************************************************************
void enablePort_1(void)
{
    P1IE |= BUTTON_1 + BUTTON_2;    // P1.0 interrupt enabled
    P1IES |= BUTTON_1 + BUTTON_2;   // P1.0 Hi/lo edge
    P1REN |= BUTTON_1 + BUTTON_2;   // Enable Pull Up
    P1IFG &= ~BUTTON_1 + ~BUTTON_2; // P1.0 IFG cleared
}

//*****************************************************************************
// int UART_sendData(char)
// Input: char
// Output: void
// Functions: Sends char over UART
//*****************************************************************************
void UART_sendData(char value)
{
    while (!(IFG2 & UCA0TXIFG));
    UCA0TXBUF = value;
}

//*****************************************************************************
// void UART_encode(int, int)
// Input: int, int
// Output: void
// Functions: Adds Parity Bit for UART transmission
//*****************************************************************************
void UART_encode(int digit, int data)
{
    switch(digit)
    {
        case 0:
            D0 = data + D0_parity;
            break;
        case 1:
            D1 = data + D1_parity;
            break;
        case 2:
            D2 = data + D2_parity;
            break;
        case 3:
            D3 = data + D3_parity;
            break;
    }
}

//**************************************************************************************************************************
// Setup Board 2
//**************************************************************************************************************************
//*****************************************************************************
// void enableGPIO2()
// Input: Void
// Output: void
// Functions: Enables GPIO Pins for MCU2
//*****************************************************************************
void enableGPIOBoard2(void)
{
    // Setting the UART function for P1.1 & P1.2
    P1SEL = RXD; // P1.1 = RXD, P1.2=TXD
    P1SEL2 = RXD; // P1.1 = RXD, P1.2=TXD
    // Configuring 1/O for display
    P1DIR |= 0x78; // Set P1.3 - P1.6 to output direction
    P2DIR |= 0xFF; // Set P2 to output direction
    P2SEL &= ~(BIT6 | BIT7); // Enable P2.1 & P2.7
}
//*****************************************************************************
// void enableUARTBoard2()
// Input: Void
// Output: void
// Functions: Enables UART communications for MCU2
//            This function enables the interrupt, while MCU1 doesn't
//*****************************************************************************
void enableUARTBoard2(void)
{
    UCA0CTL1 |= UCSSEL_2; // SMCLK
    UCA0BR0 = 104; // 1MHz 9600
    UCA0BR1 = 0; // 1MHz 9600
    UCA0MCTL = UCBRS1 + UCBRS0; // Modulation UCBRSx = 1
    UCA0CTL1 &= ~UCSWRST; // **Initialize USCI state machine**
    IE2 |= UCA0RXIE; // Enable USCI_A0 RX interrupt
}


//**************************************************************************************************************************
// Interrupts
//**************************************************************************************************************************
//*************************************************************
// UART RX interrupt service routine
//
// Function: Echo back RXed character, confirm TX buffer
// is ready first
//*************************************************************
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
    int temp = UCA0RXBUF;
    int parityCheck = temp >> 4;
    switch(parityCheck)
    {
        case 0xA:
            D0 = temp - D0_parity;
            break;
        case 0xB:
            D1 = temp - D1_parity;
            break;
        case 0xC:
            D2 = temp - D2_parity;
            break;
        case 0xD:
            D3 = temp - D3_parity;
            break;
    }
}

//*************************************************************
// Timer A0 Interrupt service routine
//
// Function: Interrupt changes current state every 30 sec
//*************************************************************
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TimerA0(void)
{
    if( ++timerCount >= 60 )            // 30 seconds
    {
        timerCount = 0;                 // Reset counter
        switch(state)                   // Switch State
        {
            case AMI:
                state = MSG;
                break;
            case MSG:
            case Presses:
                state = AMI;
                break;
        }
    }
}

//*************************************************************
// Port 1 interrupt service routine
//
// Function: Changes the state every time button 1 presses
//           Button 2 resets press count
//*************************************************************
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    __delay_cycles(0xCFF);
    if((P1IN & BUTTON_1) != BUTTON_1)
    {
        buttonPressed += 1;     // increment count
        timerCount = 0;         // Reset counter
        TA0R = 0;               // Reset Timer

        if(state == AMI) state = Presses;
        else if(state == MSG) state = Presses;
//        else if(state == Presses) state = AMI;    // Uncomment to make button 1 toggle modes
    }
    else if((P1IN & BUTTON_2) != BUTTON_2)
    {
        buttonPressed = 0;
    }
    P1IFG &= ~BUTTON_1 + ~BUTTON_2; // P1.0 IFG cleared
}

//**************************************************************************************************************************
// 7 Segment
//**************************************************************************************************************************
//*****************************************************************************
// void smallDelay()
// Input: Void
// Output: void
// Functions: Artificial delay to give processor time to respond
// "Everything is timing based" - Joey Phillips
//*****************************************************************************
void smallDelay(void)
{
    __delay_cycles(0xFFF);
}
//*****************************************************************************
// void sevenSeg(int)
// Input: int
// Output: void
// Functions: Displays decimal value on a quad seven segment LED array.
// Grabs output value from lookup table in definitions.
//
// PINS: Board Segment 7-segment Pin
// ***** **************************** *************
// P2.1 A Top Segment 11
// P2.2 B Top Right Segment 7
// P2.4 C Bottom Right Segment 4
// P2.7 D Bottom Segment 2
// P2.6 E Bottom Left Segment 1
// P2.0 F Top Left Segment 10
// P2.3 G Middle Segment 5
// P2.5 Dp Decimal Point 3
//
// Output order: 0b DEDpC GBFA
//*****************************************************************************
// Lookup tables for Seven Segment LED output values
unsigned int sevenSegDigits[] =
    {0x28, 0xEB, 0x32, 0x62, 0xE1, 0x64, 0x24, 0xEA, 0x20, 0x60, 0xA0, 0x78, 0xBD, 0x64, 0x2C, 0xFF};
    // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, A, M, I, S, G, null

void sevenSeg(int input)
{
    if(input > NULL_VAL-1) input = NULL_VAL;
    P2OUT = sevenSegDigits[input];
}

//*****************************************************************************
// void quadDigitArray(int input)
// Input: int
// Output: void
// Functions: Display decimal value on quad seven segment Led using multiplexing
//
// PINS: Board Digit Quad Array Pin
// ***** **************************** *************
// P1.6 d1 1's place 6
// P1.5 d2 10's place 8
// P1.4 d3 100's place 9
// P1.3 d4 1000's place 12
//*****************************************************************************
void quadDigitArray(int digit0, int digit1, int digit2, int buff3)
{
    // Multiplex through digits 1-4 outputting specified number
    P1OUT = DIGIT_1s; // Sets digit to display on
    sevenSeg(digit0); // Displays the digit
    smallDelay();     // Delay before moving to next digit
    P1OUT = DIGIT_10s;
    sevenSeg(digit1);
    smallDelay();
    P1OUT = DIGIT_100s;
    sevenSeg(digit2);
    smallDelay();
    P1OUT = DIGIT_1000s;
    sevenSeg(buff3);
    smallDelay();
}

//*****************************************************************************
// void digitSeparator(int)
// Input: int
// Output: void
// Functions: Breaks input into individual digits for Transmission
//*****************************************************************************
void digitSeparator(int ADC)
{
    int digit3 = ADC / 1000;
    ADC %= 1000;
    int digit2 = ADC / 100;
    ADC %= 100;
    int digit1 = ADC / 10;
    ADC %= 10;
    int digit0 = ADC;

    // Replace Leading Zeros with null value
    if(digit3 == 0 && digit2>0)
    {
        digit3 = NULL_VAL;
    }
    else if(digit3 == 0 && digit2 == 0 && digit1>0)
    {
        digit3 = NULL_VAL;
        digit2 = NULL_VAL;
    }
    else if(digit3 == 0 && digit2 == 0 && digit1 == 0)
    {
        digit3 = NULL_VAL;
        digit2 = NULL_VAL;
        digit1 = NULL_VAL;
    }
    else if(digit2 == 0 && digit1 == 0 && digit0 == 0)
    {
        digit3 = NULL_VAL;
        digit2 = NULL_VAL;
        digit1 = NULL_VAL;
        digit0 = 0;
    }

    UART_encode(0, digit0);
    UART_encode(1, digit1);
    UART_encode(2, digit2);
    UART_encode(3, digit3);
}
