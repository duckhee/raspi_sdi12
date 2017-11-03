/*================================= Arduino SDI-12 =============================== (Kevin Smith)
Arduino library for SDI-12 communications to a wide variety of environmental sensors. This library provides a general software solution, without requiring any additional hardware.
============================= Original Attribution & License ===================== (Kevin Smith)
Copyright (C) 2013 Stroud Water Research Centre Available at https://github.com/StroudCenter/Arduino-SDI-12
Authored initially in August 2013 by:
Kevin M. Smith (http://ethosengineering.org)
Inquiries: SDI12@ethosengineering.org
based on the SoftwareSerial library (formerly NewSoftSerial), authored by:
ladyada (http://ladyada.net)
Mikal Hart (http://www.arduiniana.org)
Paul Stoffregen (http://www.pjrc.com)
Garrett Mace (http://www.macetech.com)
Brett Hagman (http://www.roguerobotics.com/)
This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation;
 either version 2.1 of the License, or (at your option) any later version.
This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License along with this library; if not, write to the Free Software Foundation,
 Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
==================================== Code Organization ========================= (Kevin Smith)
0. Includes, Defines, & Variable Declarations
1. Buffer Setup
2. Data Line States, Overview of Interrupts
3. Constructor, Destructor, SDI12.begin(), and SDI12.end()
4. Waking up, and talking to, the sensors.
5. Reading from the SDI-12 object. available(), peek(), read(), flush()
6. Interrupt Service Routine (getting the data into the buffer)
*/
/* ===== 0. Includes, Defines, and Variable Declarations ======= (Kevin Smith and James Coppock)
(KMS:
0.1 - Include the header file for this library.
0.2 - defines the size of the buffer
0.3 - defines value for DISABLED state
0.4 - defines value for ENABLED state
0.5 - defines value for DISABLED state
0.6 - defines value for TRANSMITTING state
0.7 - defines value for LISTENING state
)
(JMC: 0.8 - defines value for ENABLEINTERRUPT state.)
(KMS: 0.9 - Specifies the delay for transmitting bits. (JMC: 1200 Baud equates to 833us but
with system calls the actual time was measured using an oscilloscope to be 805us.)
)
(JMC: 0.10 to 0.13 are new reference variables.
0.10 - a reference to the pin that connects to the SN74HCT240 output enable pin. This
pin puts the TX data pin on the output of the SN74HCT240 in a high impedance state.)
0.11 - a reference to the TX data pin. This pin is connected to an input of the
SN74HCT240.)
0.12 - a reference to the pin that connects to one of the SN74HCT240 output enable
pin. This pin puts the output of the SN74HCT240 connected to the RX data pin of the Pi in a high impedance state.)
0.13 - a reference to the RX data pin. This pin is connected to an output of the
SN74HCT240.)
)
(KMS: 0.14 - holds the buffer overflow status.)
(JMC: 0.15 - a new reference variable which holds the parity error status.
)
*/

#include <SDI12.h>
#define _BUFFER_SIZE           75                        //max buffer size
#define DISABLED               0                         //value for DISABLED state
#define ENABLED                1                         //value for ENABLED state
#define HOLDING                2                         //value for DISABLED state
#define TRANSMITTING           3                         //value for TRANSMITTING state
#define LISTENING              4                         //value for LISTENING state
#define INTERRUPTENABLED       5                         //(JMC: 0.8 value for ENABLEINTERRUPT state)
#define SPACING                805                       //bit timing in microseconds

uint8_t _txEnable;                                        //(JMC: refernce to the pin that connects to one of the SN74HCT240 output enable pins)
uint8_t _txDataPin;                                       //(JMC: reference to the tx data pin)
uint8_t _rxEnable;                                        //(JMC: reference to the pin that connects to one of the SN74HCT240 output enable pins)
uint8_t _rxDataPin;                                       //(JMC: reference to the rx data pin)

bool _bufferOverflow;                                    //(buffer overflow status)
bool _parityError;                                       //(parity error status)

/* ================================ 1. Buffer Setup ============================ ( Kevin Smith)
The buffer holds the ascii characters from the SDI-12 bus. Characters are read into the buffer when an interrupt is received on the data line.
 The buffer uses a circular implementation with pointers to both the head and the tail. The circular buffer is defined with the size of the buffer and two pointers;
- One to the start of data
- One to end of data
When a pointer reaches the end of the buffer it jumps back to the start.
1.1 - Define a maximum buffer size (in number of characters).
1.2 - Declare a character array of the specified size.
1.3 - Index to buffer head. (JMC: Buffer head is the index to first character in. The index is
declared as uint8_t type which is an unsigned 8-bit integer, can map from 0-255.)
1.4 - Index to buffer tail. (JMC: Buffer tail is the index to 1 position advanced of the last
character unless empty.)
*/
// See section 0.2 above. // 1.1 - max buffer size
char _rxBuffer[_BUFFER_SIZE];                           //declare an array(buffer for incoming ascii characters)
uint8_t _rxBufferHead = 0;                              //index of buff head
uint8_t _rxBufferTail = 0;                              //index of buff tail

/* ==================================== 2. Data Line States ================== ( James Coppock)
The original library specified 4 states. The original library used 1 single digital I/O pin of an Arduino. The Arduino pins are 5 volts and can supply 20 mA of current.
 The voltage of the Arduino pin is compatible with SDI-12 voltage and 20 mA is plenty to drive at least 10 sensor.
The Raspberry Pi pins are 3.3 volt and can supply 16 mA from a single pin safely (the pins are not current limited).
 The data pin(s) must be connected to some hardware to interface the Raspberry Pi to the SDI-12 bus. The best way to do this is by having two data pins,
  one TX data pin and one RX data pin and connecting to a SN74HCT240 inverting tristate buffer line driver.
All communications are initiated by the Raspberry Pi. The state of the line is set to 1 of 5 states.
RXDATAPIN RXDATAPIN RXENABLE TXDATAPIN TXENABLE
(BCM 22) (BCM 22) (BCM 27) (BCM 17) (BCM 4)
Input Input Output Output Output
State Interrupt Mode LEVEL Level Level Level
(240 1Y1) (240 1Y1) (240 1OE) (240 2A1) (240 2OE)
HOLDING Falling pulup HIGH HIGH LOW
TRANSMITTING Falling pulup HIGH Vary LOW
LISTENING Falling pulup LOW Dont Care HIGH
DISABLED Disable pulup HIGH Dont care HIGH
INTERRUPTENABLE Enable Fall Pulup HIGH HIGH LOW
* NOTE: OE HIGH means output of SN74HCT240 in high impedance state.
------------------------------------------| Sequencing |------------------------------------
INTERRUPTENABLE --> TRANSMITTING --> LISTENING --> TRANSMITTING --> LISTENING --> ..... --> DISABLED
-----------------------------------| Function Descriptions |-----------------------------------
2.1 - A private function, sets the state of 4 pins that connect to the tri state buffer and line driver with separate output enable pins. 
The 5 states are given in the table above which are HOLDING, TRANSMITTING, LISTENING, DISABLED and INTERRUPTENABLE. 
4 of the five states were defined in original code but all the code of this member function is changed. The functions used to write a HIGH or LOW on any pin
, and the interrupt enable and disable are from the wiringPi library.
2.2 - A public function which forces a "HOLDING" state. This function is called after a failed communication due to noise or to place line into
 a low impedance state before initiating communication with a sensor.
// 2.1 - sets the state of the SDI-12 object. (JMC: All setState() function code has been modified to control SN74HCT240 using wiringPi libraries as mentioned in section 2 comments above)
*/
void SDI12::setState(uint8_t state)
{
    if(state == HOLDING)                                          //if HOLDING
    {
        //std::cout << "SetState = HOLDING" << "\n";
        digitalWrite(_rxEnable, HIGH);                           //State of 240 output 1 in high impedance
        digitalWrite(_txEnable, HIGH);                           //Set TX pin HIGH level(txDataPin = BCM17)
        digitalWrite(_txDataPin, LOW);                           //Set State of 240 output 2 'driving' state
        return ;
    }
    if(state == TRANSMITTING)                                    //if TRANSMITTING
    {
        //std:cout << "SetState = TRANSMITTING" << "\n";
        digitalWrite(_rxEnable, HIGH);                           //State of 240 output 1 high impedance
        digitalWrite(_txEnable, LOW);                            //State of 240 output 2 is driving staet
        return ;
    }
    if(state == LISTENING)                                       //if LISTENING
    {
        digitalWrite(_txEnable, HIGH);                           //State of 240 output 2 (Tx output) is driving state
        digitalWrite(_rxEnable, LOW);                            //State of 240 output 1 (RX output) is high impedance
    }
    if(state == DISABLED)                                        //if state == DISABLED. pin interrupt disabled
    {
        //std::cout << "SetState = DISABLED" << "\n";
        //Only necessary to disable if using ISR routine
        system("gpio edge 22 none");                            //DIsable rising edge interrupt detection on RXDATAPIN
        digitalWrite(_rxEnable, HIGH);                          //State of 240 output 1 (RX input) is high impedance
        digitalWrite(_txEnable, HIGH);                          //State of 240 output 2 (TX output) is driving state
        return ;
    }
    if(state == INTERRUPTENABLED)                               //if state == INTERRUPT. Enables pin interrupt
    {
        //std::cout << "SetState = INTERRUPTENABLE" << "\n";
        pullUpDnControl(_rxDataPin, PUD_UP);                   //Set RX Pin with pull down resistors enabled
        system("gpiio edge 22 falling");                       //Enable rising edge interrupt detection on RXDATAPIN
        pullUpDnControl(_rxDataPin, PUD_DOWN);                 //Triggers the first interrupt which has a bug and triggers two.Both these need to be ignored.
        pullPuDnControl(_rxDataPin, PUD_UP);                   //Triggers the first interrupt which has a bug and triggers two. Both these need to be ignored.
        delay(1);
        digitalWrite(_rxEnable, HIGH);                         //State of 240 output 1 in high impedance
        digitalWrite(_txDataPin, HIGH);                        //Set Tx pin HIGH level (txDataPin = BCM17)
        digitalWrite(_txEnable, LOW);                          //Set State of 240 output 2 'driving' state
        return ;
    }
    else                                                       //Error message due to unexpected value.
    {
        std::cout << "Error: SetState = unknown check script for mistake" << "\n";
        //std::cout << state << "\n";
        std::cout << unsigned(state) << "\n";
    }
}
//forces a HOLDING state.
void SDI12::forceHold()
{
    //std::cout << "ForceHold() called" << "\n";
    setState(HOLDING);
}
/*
================================= 3. Constructor, Destructor, SDI12.begin(), SDI12.end(), parityErrorStatus(), and overflowStatus()====================== (Kevin Smith and James Coppock)
(KMS: 3.1 - The constructor requires a four parameter, which are the four pins to be used for
the data line. When the constructor is called it resets the buffer overflow status to FALSE 
(JMC: and resets the parity overflow status to false and assigns the pin numbers txEnable, txDataPin, rxEnable and rxDataPin to private variables
"_txEnable", "_txDataPin", "_rxEnable" and "_rxDataPin".)
)
(KMS: 3.2 - When the destructor is called, it's main task is to disable any interrupts that had
been previously assigned to the pin, so that the pin will behave as expected when
used for other purposes. This is achieved by putting the SDI-12 object in the
DISABLED state.
3.3 - begin() - public function called to begin the functionality of the
SDI-12 object. It has no parameters as the SDI-12 protocol is fully
specified (e.g. the baud rate is set).
3.4 - end() - public function called to temporarily cease all functionality
of the SDI-12 object. It is not as harsh as destroying the object with the
destructor, as it will maintain the memory buffer.
)
(JMC: 3.5 - parityErrorStatur() - new public function called to return the parity error status.
)
(KMS: 3.6 - overflowStatus() - public function called to return the overflow error status
)
// 3.1 Constructor (JMC: Modified function parameters)
*/
SDI12::SDI12(uint8_t txEnable, uint8_t txDataPin, uint8_t rxEnable, uint8_t rxDataPin)
{
    //std::cout << "constructor() Called \n";
    _rxBufferHead = _rxBufferTail = 0;                                          //initialise buffer pointer
    _bufferOverflow = false;                                                    //initialise buffer overflow
    _parityError = false;                                                       //(JMC: initialise parity error)
    _txEnable = txEnable;                                                       //(JMC: assign pin number to private variables)
    _txDataPin = txDataPin;                                                     //(JMC: assign pin number to private variables)
    _rxEnable = rxEnable;                                                       //(JMC: assign pin number to private variables)
    _rxDataPin = rxDataPin;                                                     //(JMC: assign pin number to private variables)
}
//Destructor
SDI12::~SDI12()
{
    //std::cout ><< "Destructor () called\n";
    setState(DISABLED);
}

//Begin - public function sets rising edge interrupt on RX datapin.
void SDI12::begin()
{
    //std::cout << "begin() called \n";
    setState(INTERRUPTENABLED);
}
//end - public function
void SDI12::end()
{
    //std::cout << "end () called \n";
    SDI12::setState(DISABLED);
}
//JMC : - new public function returns the parity error status.
bool SDI12::parityErrorStatus()
{
    //std::cout << "parityError () Called\n";
    return(_parityError);
}
//public function returns the overflow status.
bool SDI12::overflowStatus()
{
    //std::cout << "overflowStatus() Called \n";
    return(_bufferOverflow);
}
/* ========= 4. Waking up, and talking to, the sensors. =========(Kevin Smith and James Coppock)
------------------------------------| Function Descriptions |----------------------------------
(JMC: 4.1 - wakeSensors() - original function (private) that is called by the public sendCommand
function. wakeSensors() will wake the sensors on the SDI-12 bus by placing spacing
(HIGH voltage level) for a minimum of 12 milliseconds (no upper limit specified in standard). 
This is followed by a marking (logic LOW) for at least 8.33 ms (the upper limit to marking is about 90 ms). 
As the SDI-12 sensors are permitted to sleep after 100 ms of marking. Allowing some extended time on the minimum, 
the break is held for 14.161 ms and the marking for 10 ms. The state is initially set to the transmitting state.
)
(JMC: 4.2 - writeChar(uint8_t out) - slightly modified private function, that outputs a single
ASCII character to the SDI-12 bus. Each 10 bit frame that is sent out has 7 data bits, (LSB first) 1 start bit, 1 parity bit (even parity), and 1 stop bit. 
The SDI-12 protocol uses negative logic. An example a transmission of character 'a' is
shown below. The binary representation of ASCII 'a' is 110 0001.
The SDI12 line voltage for char 'a' is;
_ _ _ _ _ _
|s|d|d d d d|d d e f|
_| |_| |_ _ _ _|
d = 7 data bits
s = 1 start bit
e = 1 parity bit
f = 1 stop bit
)
There are four steps to the transmission
(JMC: 4.2.1 -The original code used an a function from the parity.h header to calculate the
parity. I have used an alternate algorithm. The algorithm calculates the number of
1's in the 8 bit frame with one parity bit and seven data bits. The algorithm returns
_evenParityBit is either 0 (even number of 1's) or 1 (odd number of 1's). The frame
should have an even number of ones for even parity.
The code merges the first 4 bits with the last 4 bits using an XOR operation. As can
be seen below the parity of two bits is computed with an XOR operation.
(0 XOR 0) -> 0
(0 XOR 1) -> 1
(1 XOR 0) -> 1
(1 XOR 1) -> 0
Now with four bits we are left with 16 possible values for _evenParityBit. Shifting
0x6996 to the right by _evenParityBit number of times leaves the relevant bit in bit
position 0. A 0 for even and a 1 for odd parity.
)
(JMC: 4.2.2 - slightly modified code to send the start bit. The start bit is a 1 on the SDI12
data line. The original code sent HIGH however writing a LOW to the TX data pin will
cause the SN74HCT240 to output a HIGH, so a LOW is written to the TX data pin for 820
us.
)
(KMS: 4.2.3 - (JMC: Slightly modified code switches the HIGH for LOW and LOW for HIGH)
Send the payload (the 7 character bits and the parity bit) least significant bit
first. This is accomplished bitwise AND operations on a moving mask (00000001) -->
(00000010) --> (00000100)... and so on. This functionality makes use of the '<<='
operator which stores the result of the bit-shift back into the left hand side.
If the result of (out & mask) determines whether a 1 or 0 should be sent.
if(out & mask){
digitalWrite(_txDataPin, HIGH);
}
else{
digitalWrite(_txDataPin, LOW);
}
4.2.4 - Slightly modified code to send the stop bit. The stop bit is always a '0', so we
Appendix C: SDI12 C++ Library 103
simply write the dataPin HIGH for 820 microseconds.
4.3 - sendCommand(String cmd) - original public function that sends out a String byte by
byte the data line.
)
*/
// 4.1 - private function that wakes up the entire sensor bus.
void SDI12::wakeSensors()
{
    setState(TRANSMITTING);
    digitalWrite(_txDataPin, LOW);
    delayMicroseconds(14161);
    digitalWrite(_txDataPin, HIGH);
    delayMicroseconds(10000);
}
//private fuctionp that writes a character out on the data line (JMC: function modified)
void SDI12::writeChar(uint8_t out)
{
    //std::cout << "Next Character out writeChar\n";
    //std::cout << out << "\n"; //1byte with 7 bit ASCII character and with even parity bit in MSB
    uint8_t _evenParityBit;                                       // (JMC: code written to calculate the parity bit)
    _evenParityBit = out;                                         // (JMC: code written to calculate the parity bit)
    _evenParityBit ^= out >> 4;                                   // (JMC: code written to calculate the parity bit)
    _evenParityBit &= 0x0F;                                       // (JMC: code written to calculate the parity bit)
    _evenParityBit = ((0x6996 >> _evenParityBit) & 1);            // (JMC: code written to calculate the parity bit)

    out |= (_evenParityBit<<7);

    digitalWrite(_txDataPin, LOW);                                // 4.2.2 - start bit
    delayMicroseconds(820);
    for (uint8_t mask = 0x01; mask; mask<<=1){                    // 4.2.3 - send payload
    if(out & mask){
    digitalWrite(_txDataPin, HIGH);
    }
    else{
    digitalWrite(_txDataPin, LOW);
    }
    delayMicroseconds(SPACING);
    }
    digitalWrite(_txDataPin, HIGH);                               // 4.2.4 - stop bit
    delayMicroseconds(820);

}
//public function that sends out the characters of the String cmd, one by one
void SDI12::sendCommand(std::string cmd)
{
    //std::cout << "snedCommand Called\n";
    wakeSensors();                                              //wake up sensors
    for(unsigned i = 0; i < cmd.length(); i++)
    {
        //std::cout << "Next Character out - sendCommand() \n";
        //std::cout << cmd[i] << "\n";                         //outputs variable(Note cout << (unsigned char))
        writeChar(cmd[i]);                                     //write each characters
    }
    setState(LISTENING);                                       //listen for reply
}

/* ============= 5. Reading from the SDI-12 object. ============(Kevin Smith and James Coppock)
(KMS: 5.1 - available() - (JMC: original public function that) returns the number of characters
available in the buffer. To understand how:
_rxBufferTail + _BUFFER_SIZE - _rxBufferHead) % _BUFFER_SIZE;
accomplishes this task, we will use a few examples.
To start take the buffer below that has _BUFFER_SIZE = 10. The message "abc" has
been wrapped around (circular buffer).
_rxBufferTail = 1 // points to the '-' after c
_rxBufferHead = 8 // points to 'a'
[ c ] [ - ] [ - ] [ - ] [ - ] [ - ] [ - ] [ - ] [ a ] [ b ]
The number of available characters is (1 + 10 - 8) % 10 = 3
The '%' or modulo operator finds the remainder of division of one number by another.
 In integer arithmetic 3 / 10 = 0, but has a remainder of 3. We can only get the remainder by using the the modulo '%'. 3 % 10 = 3. 
 This next case demonstrates more clearly why the modulo is used.
rxBufferTail = 4 // points to the '-' after c
_rxBufferHead = 1 // points to 'a'
[ a ] [ b ] [ c ] [ - ] [ - ] [ - ] [ - ] [ - ] [ - ] [ - ]
The number of available characters is (4 + 10 - 1) % 10 = 3
If we did not use the modulo we would get either ( 4 + 10 - 1 ) = 13 characters
or ( 4 + 10 - 1 ) / 10 = 1 character. Obviously neither is correct.
If there has been a buffer overflow, available() will return -1.
)
(JMC: 5.2 - new public function that checks the last character in the buffer is a <LF> without
consuming. The buffer tail is the index to 1 position advanced of the last
character in thus the last char is _rxBufferTail-1 LF = 0000 1010 = 10(dec)
)
(JMC: 5.3 - new public function that checks the second last character in the buffer is a <CR>
without consuming. The buffer tail is the index to 1 position advanced of the
last character in thus the second last char is:
_rxBufferTail-2 CR = 0000 1101 = 13(dec)
)
(KMS: 5.4 - peek() - (JMC: original public function that) allows the user to look at the
character that is at the head of the buffer. Unlike read() it does not consume the character (i.e. the index addressed by _rxBufferHead is not changed). 
peek() returns -1 if there are no characters to show.
5.5 - flush() is a modified public function that clears the buffers contents by setting
the index for both buffer head and tail back to zero. (JMC: new code also resets the status of the buffer overflow and parity error variables.)
5.6 - read() returns the character at the current head in the buffer after incrementing
the index of the buffer head. This action 'consumes' the character, meaning it cannot be read from the buffer again. If you would rather see the character, 
but leave the index to head intact, you should use peek();
)
(JMC: 5.7 - advanceBufHead() - new public function that advances the buffer head. This function
is used if only a certain part of the response from the sensor is needed. It saves reading all characters into the program and then discarding them.
)
*/
// 5.1 - public function that reveals the number of characters available in the buffer -
int SDI12::availabe()
{
    //std::cout << "available() called \n";
    if(_bufferOverflow)
    {
        return -1;
    }
    return (_rxBufferTail + _BUFFER_SIZE - _rxBufferHead) % _BUFFER_SIZE;
}

//(JMC: 5.2 - new public function that checks the last character in the buffer is a <LF> without consuming)
bool SDI12::LFCheck()
{
    //std::cout << "LFCheck() called\n";
    if(_rxBufferHead == _rxBufferTail)                             //Empty buffer? If yes, 0
    {
        return false;
    }
    int LF = _rxBuffer[_rxBufferTail -1];
    //otherwise, read from "tail" (last character in)
    if(LF == 10)
    {
        //std::cout << "Last character is a linefead <LF>! \n";
        return true;
    }
    return false;
}
//(JMC: 5.3 - new public function that checks the second last character in the buffer is a <CR> without consuming)
bool SDI12::CRCheck()
{
    //std::cout << "CRCheck() called \n";
    if(_rxBufferHead == _rxBufferTail)                          //Empty buffer? if yes, 0
    {
        return false;
    }
    int CR = _rxBuffer[_rxBufferTail - 2];
    if(CR == 13)                                                //Otherwise, check the second last character in buffer
    {
        return true;
    }
    return false;
}

//5.4 - public function that rev eals the next character in the buffer without consuming
int SDI12::peek()
{
    if(_rxBufferHead == _rxBufferTail)                        //Empty buffer? If yes, 01
    {
        return -1;
    }
    return _rxBuffer[_rxBufferHead];                          //otherwise, read from "head"
}

//5.5 - a public function that clears the buffer contents, resets the status of the buffer overflow and parrity error variables.
void SDI12::flush()
{
    //std::cout << "flush() called \n";
    _rxBufferHead = _rxBufferTail = 0;
    _bufferOverflow = false;
    _parityError = false;
}

//5.6 - read in the next characterr from the buffer and moves the index ahead. (JMC: this is FIFO opperation)
int SDI12::read()
{
    _bufferOverflow = false;
    //reading makes room in the buffer
    if(_rxBufferHead == _rxBufferTail)                     //Empty buffer ? If yes, -1
    {
        return -1;
    }
    uint8_t nextChar = _rxBuffer[_rxBufferHead];           //otherwise, grab char at head
    _rxBufferHead = (_rxBufferHead + 1) % _BUFFER_SIZE;    //increment head. modulo will reset the _rxBufferHead to 0 when _rxBufferHead = bufferSize -1
    return nextChar;                                       //return the char
}

//(JMC: 5.7 - new public function that adcvances the buffer head)
void SDI12::advanceBufHead(int advance)
{
    //std::cout << "advanceBufHead() called \n";
    //std::cout << "initial buffer head position" << (int)_rxBufferHead << "\n";
    _rxBufferHead = _rxBufferHead + advance;
    //std::cout << "new buffer head position" < (int)_rxBufferHead << "\n";
}
/* ================== 6. Interrupt Service Routine ============= ( James Coppock & Kevin Smith )
(JMC:
The original receiveChar() function did not include a parity check. I have modified the
code to include a parity error check. Most of the timing delays were changed to decrease
the chance of missing a bit when the operating system de-scheduled the thread.
wiringPi library functions used:
int digitalRead(int pin) - function returns the value read on the given pin. It will be
HIGH or LOW (1 or 0) depending on the logic level at the pin (Gordons Projects 2015).
)
(KMS:
We have received an interrupt signal, what should we do?
6.1 - function passes of responsibility to the receiveChar() function.
6.2 - This function quickly reads a new character from the data line in to the buffer.
It takes place over a series of key steps.
6.2.1 - Check for the start bit. If it is not there, interrupt may be from interference
or an interrupt we are not interested in, so return.
6.2.2 - Make space in memory for the new character "newChar".
6.2.3 - Wait half of a SPACING to help centre on the next bit. It will not actually be
centred, or even approximately so until delayMicroseconds(SPACING) is called
again.
6.2.4 - For each of the 8 bits in the payload, read whether or not the line state is
HIGH or LOW. We use a moving mask here, as was previously demonstrated in the writeByte() function.
The loop runs from i=0x1 (hexadecimal notation for 00000001) to i<0x80
(hexadecimal notation for 10000000). So the loop effectively uses the
masks following masks: 00000001
00000010
00000100
00001000
00010000
00100000
01000000 and their inverses.
Here we use an if / else structure that helps to balance the time it takes to
either a HIGH vs a LOW, and helps maintain a constant timing.
6.2.5 - Skip the stop bit.
(JMC:
6.2.6 - The original code skipped the parity bit with a delay of 830 microseconds. Due
to the number of parity errors that would come up the parity check was needed.
The algorithm calculates the number of 1's in the 8 bit frame with one parity
bit and seven data bits. The algorithm returns _evenParityBit is either 0 (even
number of 1's) or 1 (odd number of 1's). The frame should have an even number of ones for even parity.
The code merges the first 4 bits with the last 4 bits using an XOR
operation. As can be seen below the parity of two bits is computed with
an XOR operation.
(0 XOR 0) -> 0
(0 XOR 1) -> 1
(1 XOR 0) -> 1
(1 XOR 1) -> 0
Now with four bits we are left with 16 possible values for _evenParityBit Shifting 0x6996 to the right by _evenParityBit number of times leaves the
relevant bit in bit position 0. 6996 (Hex) = 0110 1001 1001 0110 (bin) which gives the 16 possibilities for the parity. A 0 for even and a 1 for odd parity
If a parity error is picked up parityError status is set to true and the state
is set to disabled. The interrupts will be disabled also.
)
(KMS:
6.2.7 - Check for an overflow. Check if advancing the index to most recent buffer entry
(tail) will make it have the same index as the head (in a circular fashion). If there is an overflow a character will not be stored and hence will not overwrite buffer head
6.2.8 - Save the byte into the buffer if there has not been an overflow, and then
advance the tail index.
)
*/

// 6.1 - public static function that passes off responsibility for an interrupt to the receiveChar() function.
inline void SDI12::handleInterrupt()
{
    if(_parityError == true)
    {
        std::cout << "handleInterrupt() error : parity error is true: \n";
        return ;
    }
    receiveChar();
}

//6.2 - private function that reads a new character into the buffer(JMC : function modified to do parity error check on each received)
inline void SDI12::receiveChar()
{
    //std::cout << "receiveChar() called \n";

    if(digitalRead(_rxDataPin) == 0)                                  //6.2.1 - Is the start bit LOW? a HIGH indicates a false trigger of interrupt
    {
        uint8_t newChar = 0;                                          //6.2.2 - Declare and initialise variable for char.
        delayMicrosecnds(20);                                         //6.2.3 - sets a small delay period after the falling edge of the start bit was detected
        for(uint16_t i = 0x1; i <= 0x80; i <<= 1)                     //6.2.4 - read the 7 data bits (for i = 1 to 0100 0000 (<<= bitshift assignment))
        {
            delayMicroseconds(800);                                   //(800)  Delay 800 us. This seems to work better than a full symbol period of 830 us.
            uint8_t noti = ~i;                                        //~Bitwise NOT operator
            if(!digitalRead(_rxDataPin))                              //If pin level is LOW(NOTE ! is Logical NOT operator)
            {
                std::cout << "pin level LOW : " << "\n";
                newChar &= noti;
            }
            else{                                                     //else pin level is HIGH
                std::cout << "pin level HIGH : " << "\n";
                newChar |= i;                                         // |= Bitwise inclusive OR assignment operator
            }
        }

        uint8_t newChar2 = newChar;
        newChar2 &= 0x7F;

        delayMicroseconds(650);                                     //(650) 6.2.5 - Is the stop bit LOW? a LOW indicates an incorrect stop bit (inverted logic)
        if(digitalRead(_rxDataPin) == 0)
        {
        std::cout << "receiveChar() Incorrect stop bit : - parityError set to true and interrupt disabled \n";
        _parityError = true;                                        //JMC:
        SDI12::end();                                               //JMC: Disable interrupt
        return;                                                     //JMC:
    }

        // (JMC: 6.2.6 - Check for parity error.)
        uint8_t _evenOrOdd;                                         //JMC
        _evenOrOdd = newChar;                                       //JMC
        _evenOrOdd ^= newChar >> 4;                                 //JMC
        _evenOrOdd &= 0x0F;                                         //JMC
        _evenOrOdd = ((0x6996 >> _evenOrOdd) & 1);                  //JMC
        std::cout << "receiveChar() parity error: - parityError set to true - check parityError()\n";
        _parityError = true;
        SDI12::end();
        return ;
    }

    newChar &= 0x7F;                                               //Set the most significant bit(parity bit) to 0 leaving the 7 bit ASCII character.

    if((_rxBufferTail + 1) == _rxBufferHead)                       //6.2.7 - Overflow? If not, proceed.
    {
        _bufferOverflow = true;                                    //bufferOverflow status set and newChar is not stored
        std::cout << "Buffer full - check overflowStatus() " << "\n";
    }else{                                                         //save char, advance tail.
        _rxBuffer[_rxBufferTail] = newChar;
        _rxBufferTail = (_rxBufferTail + 1) % _BUFFER_SIZE;        //increments buffer tail and resets to 0 if _rxBufferTail+1 == BUFFERSIZE
    }
}
