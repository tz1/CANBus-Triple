#include <SPI.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>

// MCP2515 SPI commands
#define RESET_REG 0xc0
#define READ 0x03
#define WRITE 0x02              //read and write comands for SPI
#define READ_RX_BUF_0_ID 0x90   // 0_DATA 0x92, 1_ID 0x94, 1_DATA 0x96
#define LOAD_TX_BUF_0_ID 0x40   // 0_DATA 0x41, 1_ID 0x42, 1_DATA 0x43, 2_ID 0x44, 2_DATA 0x45
#define SEND_TX_BUF_0 0x81      // _1 0x82, _2 0x83
#define READ_STATUS 0xA0
#define RX_STATUS 0xB0
#define BIT_MODIFY 0x05         //Other commands

//Registers
#define RXERCNT 0x1d            //receive error count
#define CNF3 0x28
#define CANCTRL 0x0F            //Mode control register
#define CANINTE 0x2B            // Interrupt Enable
#define CANINTF 0x2c            // Interrupt flags (pending)
// merr, wake err tx2 tx1 tx0 rx1 rx1
#define EFLG 0x2D               // Error Register address

#define BOOT_LED 13

#define NCAN 3

#define CAN1RESET 4
#define CAN1INT 0 //D3 // int0
#define CAN1SELECT 9

#define CAN2RESET 12
#define CAN2INT 1 //D2 // int1
#define CAN2SELECT 10

#define CAN3RESET 11
#define CAN3INT 4 //D7 // int.6
#define CAN3SELECT 5
static byte can_reset[NCAN] = { CAN1RESET, CAN2RESET, CAN3RESET };
static byte can_ss[NCAN] = { CAN1SELECT, CAN2SELECT, CAN3SELECT };

#include <avr/io.h>

// 8MHz SPI Read
static void recvspiblock(byte *buf, unsigned short len)
{
  if (SPSR & 1) {
    byte t;
    SPDR = 0xff;
    while (--len) {
      asm(" .rept 9\n nop\n .endr");
      t = SPDR;
      SPDR = 0xff;
      // there is actuallya race condition if an interrupt occurs here.
      *buf++ = t;         // double buffered
    }
    while (!(SPSR & 0x80));
    SPSR &= 0x7f;
    *buf = SPDR;
  }
}

#ifdef SRAM
static byte spixbuf[16], spixlen;

static void sendspiblock(byte *buf, unsigned short len)
{
  if (SPSR & 1) {
    // NOTE you cannot reduce the number of NOPs since they are counting SPI clocks, not a delay
    while (len--) {
      SPDR = *buf++;
      asm(" .rept 11\n nop\n .endr");
    }
    return;
  }
  volatile byte x;
  while (len--) {
    SPDR = *buf++;
    while (!(SPSR & 0x80));
    SPSR &= 0x7f;
    x = SPDR;
  }
}

#define SRAMSELECT 6 // pd7
#define SRAMREAD 3
#define SRAMWRITE 2

static void sramread(unsigned long addr, byte *buf, unsigned short len )
{
  digitalWrite(SRAMSELECT, LOW);
  SPI.transfer(SRAMREAD);
  SPI.transfer(addr >> 16);
  SPI.transfer(addr >> 8);
  SPI.transfer(addr);
  recvspiblock(buf,len);  
  digitalWrite(SRAMSELECT, HIGH);
}

static void sramwrite(unsigned long addr, byte *buf, unsigned short len )
{
  digitalWrite(SRAMSELECT, LOW);
  SPI.transfer(SRAMWRITE);
  SPI.transfer(addr >> 16);
  SPI.transfer(addr >> 8);
  SPI.transfer(addr);
  sendspiblock(buf,len);  
  digitalWrite(SRAMSELECT, HIGH);
}
#endif
static byte canstatus(byte chan)
{
  byte retVal;
  digitalWrite(can_ss[chan], LOW);
  SPI.transfer(READ_STATUS);
  retVal = SPI.transfer(0xFF);
  digitalWrite(can_ss[chan], HIGH);
  return retVal;
}

static bool cansend(byte chan, unsigned long ident, bool extid, byte len, byte * data)
{
  byte i, id_high, id_low, id_exth, id_extl;

  byte stat = canstatus(chan) & 0x54;
  if (stat == 0x54)
    return false;           // all buffers full;
  byte ch = 0;
  if (stat & 0x4) {           // ch 0 busy
    ch = 1;
    if (stat & 0x10)        // ch 1 busy too
      ch = 2;
  }
  byte bufid[6];
  memset(bufid, 6, 0);
  bufid[0] = LOAD_TX_BUF_0_ID + ch + ch;
  bufid[5] = len;
  byte txbuf = SEND_TX_BUF_0 + ch;

  //generate id bytes before SPI write
  if (extid) {
    bufid[1] = ident >> 21;
    bufid[2] = ((ident >> 13) & 0xE0) | 8 | ((ident >> 16) & 3);
    bufid[3] = ident >> 8;
    bufid[4] = ident;
  }
  else {
    bufid[1] = (ident >> 3);
    bufid[2] = ((ident << 5) & 0xE0);
  }
  digitalWrite(BOOT_LED, HIGH);

  digitalWrite(can_ss[chan], LOW);
  for (i = 0; i < 6; i++)     //load data buffer
    SPI.transfer(bufid[i]);
  for (i = 0; i < len; i++)   //load data buffer
    SPI.transfer(data[i]);
  digitalWrite(can_ss[chan], HIGH);
  digitalWrite(can_ss[chan], LOW);
  SPI.transfer(txbuf);
  digitalWrite(can_ss[chan], HIGH);

  digitalWrite(BOOT_LED, LOW);
  return true;
}

#define CANBUFSIZ 16
byte canmsgbuf[CANBUFSIZ][16];
byte canmsghead = 0, canmsgtail = 0;
byte rxcnt[NCAN];
static void canread(byte chan)
{
  byte rxstatus = canstatus(chan) & 3;
  if (!rxstatus)
    return;
//  digitalWrite(BOOT_LED, HIGH);
  byte bufsel;
  byte which = READ_RX_BUF_0_ID;
  for (bufsel = 1; bufsel < 3; bufsel <<= 1, which += 4) {
    if (!(rxstatus & bufsel))
      continue;
    byte *finf = canmsgbuf[canmsghead++];
    if (canmsghead >= CANBUFSIZ)
      canmsghead = 0;
    *finf++ = chan;
    digitalWrite(can_ss[chan], LOW);
    SPI.transfer(which);
    recvspiblock(finf, 13);
    digitalWrite(can_ss[chan], HIGH);
    rxcnt[chan]++;
  }
//  digitalWrite(BOOT_LED, LOW);
}

// Time Quanta (prescaling) with values for each closest to 80% as per SAE spec
//PROGMEM prog_uchar quantimings[] = {
PROGMEM const unsigned char quantimings[] = {
  2, 1, 1, 0,                 //8 SP@75%
  2, 2, 1, 0,
  2, 3, 1, 0,                 //10 @80
  2, 4, 1, 1,
  2, 5, 1, 1,
  2, 5, 2, 1,
  3, 5, 2, 1,
  4, 5, 2, 1,                 //15 @80
  5, 5, 2, 1,                 //16 @81.25
  6, 5, 2, 2,
  7, 4, 3, 2,
  7, 5, 3, 2,
  7, 6, 3, 2,                 //20 @ 80
  7, 7, 3, 2,
  7, 7, 4, 2,
  7, 7, 5, 2,
  7, 7, 6, 3,
  7, 7, 7, 3,
};

//CANBus triple clkout
#define CAN_XTAL 16000000

// 1 + propseg + 1 + phaseseg1 + 1 [@SP] + phaseseg2 + 1 == TotalTimeQuanta
static int canbaud(byte chan, unsigned long bitrate, byte quanta = 16)  //sets bitrate for CAN node
{
  if( quanta < 8 || quanta > 25 )
    return -1;
  byte propseg = pgm_read_byte_near(quantimings + ((quanta - 8) << 2));
  byte phaseseg1 = pgm_read_byte_near(quantimings + ((quanta - 8) << 2) + 1);
  byte phaseseg2 = pgm_read_byte_near(quantimings + ((quanta - 8) << 2) + 2);
  byte syncjump = pgm_read_byte_near(quantimings + ((quanta - 8) << 2) + 3);

  unsigned long rate = (CAN_XTAL / 2) / quanta;
  rate /= bitrate;
  if( rate > 64 )
    return 1;

// Add precision check, rate * bitrate * quanta within 1% of bitrate

  byte cnf1, cnf2;
  cnf1 = rate - 1;
  if( syncjump > 1 )
    cnf1 |= (syncjump - 1) << 6;
  cnf2 = 0x80 | phaseseg1 << 3 | propseg;

  digitalWrite(can_ss[chan], LOW);
  SPI.transfer(WRITE);
  SPI.transfer(CNF3);
  SPI.transfer(phaseseg2);
  SPI.transfer(cnf2);
  SPI.transfer(cnf1);
  digitalWrite(can_ss[chan], HIGH);
  return 0;
}

enum canmode {
  NORMAL = 0,
  SLEEP = 0x20,
  LOOPBACK = 0x40,
  LISTEN = 0x60,
  CONFIGURATION = 0x80,
};

//Method added to enable testing in loopback mode.(pcruce_at_igpp.ucla.edu)
static void canmode(byte chan, byte mode)       //put CAN controller in one of five modes
{
  digitalWrite(can_ss[chan], LOW);
  SPI.transfer(BIT_MODIFY);
  SPI.transfer(CANCTRL);
  SPI.transfer(0xe7);
  mode &= ~7;
  if ( !chan )
    mode |= 4;
  SPI.transfer(mode);
  digitalWrite(can_ss[chan], HIGH);
}

#define USEINTS
#ifdef USEINTS
// Enable / Disable interrupt pin on message Rx
static void canrxinte(byte chan, bool enable)
{
  digitalWrite(can_ss[chan], LOW);
  SPI.transfer(BIT_MODIFY);
  SPI.transfer(CANINTE);
  SPI.transfer(3);
  SPI.transfer(enable ? 3 : 0);
  digitalWrite(can_ss[chan], HIGH);
}

static void intcanrx0()
{
  canread(0);
}
static void intcanrx1()
{
  canread(1);
}
static void intcanrx2()
{
  canread(2);
}
#endif

//  note 83(.3)k, 33(.3)k and others are also doable
static unsigned long curbusrate[3] = { 500001, 250001, 125001 };

static void setcan(byte chan, unsigned long rate)
{
  // set the slaveSelectPin as an output
  byte ss = can_ss[chan];
  byte reset = can_reset[chan];
  pinMode(ss, OUTPUT);
  pinMode(reset, OUTPUT);
  digitalWrite(reset, LOW);   /* RESET CAN CONTROLLER */
  delay(50);
  digitalWrite(reset, HIGH);
  delay(100);
  if (!canbaud(chan, rate & ~1))
    curbusrate[chan] = rate;
  else
    canbaud(chan, 500001);
// AUTOBAUD - use NORMAL for 
  if( curbusrate[chan] & 1 )
    canmode(chan, LISTEN);
  else
    canmode(chan, NORMAL);
#ifdef USEINTS
  canrxinte(chan, true);
#endif
}

static void checkautobaud(byte chan)
{
  if( !(curbusrate[chan] & 1 ))
    return;
    
  digitalWrite(can_ss[chan], LOW);
  SPI.transfer(READ_STATUS);
  byte rxerc = SPI.transfer(0xFF);
  digitalWrite(can_ss[chan], HIGH);
  
  if( rxerc < 8 && rxcnt[chan] > 8 ) {
    curbusrate[chan] &= ~1;
    canmode(chan, NORMAL);
    return;
  }
  if( curbusrate[chan] < 200000 )
    curbusrate[chan] = 500000;
  else
    curbusrate[chan] >>= 1;
  setcan( chan, curbusrate[chan] );
  curbusrate[chan] |= 1;
}

#define BT_RESET TXLED //PD5
#define BT_SLEEP 8
void setup()
{
  Serial.begin(115200); // USB
  Serial1.begin(57600); // Bluetooth
  pinMode(BT_SLEEP, OUTPUT);
  digitalWrite(BT_SLEEP, HIGH);
  PORTD |= 1<<5;
  DDRD |= 1<<5;
  pinMode(BOOT_LED, OUTPUT);
  digitalWrite(BOOT_LED, LOW);

  // SPI setup
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV2); // 8MHz
  SPI.setBitOrder(MSBFIRST);
  
#ifdef SRAM
  // setup SPI RAM
  pinMode(SRAMSELECT, OUTPUT);
#endif

  // setup CANs
  byte j;
  for (j = 0; j < NCAN; j++) {
    rxcnt[j] = 0;
    //cb = EEPROM.read(j<<2);
    curbusrate[j] = 500001;    //canrates[cb];
    //curbusrate[j] = 250000;    //canrates[cb];
  }

  setcan(0, curbusrate[0]);
  setcan(1, curbusrate[1]);
  setcan(2, curbusrate[2]);

#ifdef USEINTS
  attachInterrupt(CAN1INT, intcanrx0, LOW);
  attachInterrupt(CAN2INT, intcanrx1, LOW);
  attachInterrupt(CAN3INT, intcanrx2, LOW);
#endif

//  strcpy((char *)spixbuf, "Hello World!");
//  sramwrite( 0, spixbuf, 13 );
//  memset( spixbuf, 0, 16 );
//  cansend(2, 0x08880808, true, 8, (byte *) "HelloCAN");
}

#ifdef EMIT_B64
/*prog_ */ char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
// bus:2 dlc:4
// extid:1 highest:5
// for extid (hi-mid 6) (mid 6) (lo-mid 6)
// lowest:6
// data, 0, 2==, 3=, 4, 6==, 7=, 8, 10==, 11=
#endif

static void printcanrx()
{
  while (canmsghead != canmsgtail) {

    byte chan, *frameinfo = canmsgbuf[canmsgtail++];

    if (canmsgtail >= CANBUFSIZ)
      canmsgtail = 0;

    chan = *frameinfo++;

    Serial.print(chan);
    unsigned long id = 0;
    for (int j = 0; j < 4; j++) {
      id <<= 8;
      id |= frameinfo[j];
    }
    if (frameinfo[1] & 8) { // extended
      Serial.print(" x");
      unsigned long id2 = id;
      id2 &= 0x3ffff;
      id &= 0xffe00000;
      id >>= 3;
      id |= id2;
    }
    else {
      Serial.print(" s");
      id >>= 21;
    }
    Serial.print(id, HEX);
    for (int j = 0; j < (frameinfo[4] & 0x0f); j++) {
      Serial.print(" ");
      if (frameinfo[j + 5] < 0x10)
        Serial.print("0");
      Serial.print(frameinfo[j + 5], HEX);
    }
    Serial.println();
#ifdef EMIT_B64
    char b64enc[20], *benc = b64enc;
    *benc++ = b64[(chan << 4) | (frameinfo[4] & 0x0f)];

    if (frameinfo[1] & 8) { // extended
      *benc++ = b64[32 | (id >> 24)];
      *benc++ = b64[0x3f & (id >> 18)];
      *benc++ = b64[0x3f & (id >> 12)];
      *benc++ = b64[0x3f & (id >> 6)];
      *benc++ = b64[0x3f & id];
    }
    else {
      *benc++ = b64[0x1f & (id >> 6)];
      *benc++ = b64[0x3f & id];
    }
    byte more, mbits = 0;
    for (int j = 0; j < (frameinfo[4] & 0x0f); j++) {
      switch (mbits) {
        case 0:
          *benc++ = b64[frameinfo[j + 5] >> 2];
          mbits = 2;
          more = (frameinfo[j + 5] & 3) << 4;
          break;
        case 2:
          *benc++ = b64[more | (frameinfo[j + 5] >> 4)];
          mbits = 4;
          more = (frameinfo[j + 5] & 0xf) << 2;
          break;
        case 4:
          *benc++ = b64[more | (frameinfo[j + 5] >> 6)];
          *benc++ = b64[frameinfo[j + 5] >> 2];
          mbits = 0;
          break;
      }
    }
    if (mbits) {
      *benc++ = b64[more];
      // "correct" b64 with equals termination
      *benc++ = '=';
      if (mbits == 2)
        *benc++ = '=';
    }
    *benc = 0;
    Serial.println(b64enc);
#endif
  }
}

void loop()
{
#ifndef USEINTS
  canread(0);
  canread(1);
  canread(2);
#endif  
  printcanrx();
#if 1  
static unsigned long x;  
  if (!(x++ & 0xfffff)) {
  checkautobaud(0);
  checkautobaud(1);
  checkautobaud(2);
  }
  #endif
//  sramread( 0, spixbuf, 13 );
//  Serial.println((const char *)spixbuf);
  while(Serial.available()) Serial1.write(Serial.read());
  while(Serial1.available()) Serial.write(Serial1.read());

#if 0
  static unsigned long x;
  if (!(x++ & 65535))
    cansend(2, 0x08880808, true, 8, (byte *) "HelloCAN");
  if (!((x ^ 32768) & 65535))
    cansend(2, 0x7E8, false, 8, (byte *) "@UU@U@@U");
#endif
}
