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
#define CNF3 0x28
#define CANCTRL 0x0F            //Mode control register
#define CANINTE 0x2B            // Interrupt Enable
#define CANINTF 0x2c            // Interrupt flags (pending)
// merr, wake err tx2 tx1 tx0 rx1 rx1
#define EFLG 0x2D               // Error Register address

#define BOOT_LED 13

#define NCANS 3
static byte can_ss[NCANS];      // slave selects
static byte spixbuf[16], spixlen;

#if 1
#include <avr/io.h>
static void sendspiblock()
{
  byte *buf = spixbuf, len = spixlen;
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

static void recvspiblock(unsigned short len)
{
  byte *buf = spixbuf;
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
  while (len--) {
    SPDR = 0xff;
    while (!(SPSR & 0x80));
    SPSR &= 0x7f;
    *buf++ = SPDR;
  }
}

void x() {
  SPCR = 0x50;
  SPI.setClockDivider(0);

}
#endif



static byte canstatus(byte bid)
{
  byte retVal;
  digitalWrite(can_ss[bid], LOW);
  SPI.transfer(READ_STATUS);
  retVal = SPI.transfer(0xFF);
  digitalWrite(can_ss[bid], HIGH);
  return retVal;
}

static bool cansend(byte bid, unsigned long ident, bool extid, byte len, byte * data)
{
  byte i, id_high, id_low, id_exth, id_extl;

  byte stat = canstatus(bid) & 0x54;
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

  digitalWrite(can_ss[bid], LOW);
  for (i = 0; i < 6; i++)     //load data buffer
    SPI.transfer(bufid[i]);
  for (i = 0; i < len; i++)   //load data buffer
    SPI.transfer(data[i]);
  digitalWrite(can_ss[bid], HIGH);
  digitalWrite(can_ss[bid], LOW);
  SPI.transfer(txbuf);
  digitalWrite(can_ss[bid], HIGH);

  digitalWrite(BOOT_LED, LOW);
  return true;
}

#define CANBUFSIZ 16
byte canmsgbuf[CANBUFSIZ][16];
byte canmsghead = 0, canmsgtail = 0;

static void canread(byte bid)
{
  byte rxstatus = canstatus(bid) & 3;
  if (!rxstatus)
    return;

  digitalWrite(BOOT_LED, HIGH);
  byte bufsel;
  byte which = READ_RX_BUF_0_ID;

  for (bufsel = 1; bufsel < 3; bufsel <<= 1, which += 4) {
    if (!(rxstatus & bufsel))
      continue;
    byte *finf = canmsgbuf[canmsghead++];
    if (canmsghead >= CANBUFSIZ)
      canmsghead = 0;
    *finf++ = bid;
    digitalWrite(can_ss[bid], LOW);
    SPI.transfer(which);
#if 1
    recvspiblock(13);
    memcpy( finf, spixbuf, 5 + (spixbuf[4] & 0x0f) );
#else
    byte j;
    for (j = 0; j < 5; j++)
      *finf++ = SPI.transfer(0xFF);
    //id high //id low plus ext  S2S1S0 SRR EXTID/STDID x EID17 EID16
    //extended id 15..8 //extended id 7..0
    // x, RTRxF, rsv, rsv, DLC:4
    byte len = finf[-1] & 0x0F;     //data length code
    while (len--)
      *finf++ = SPI.transfer(0xFF);
#endif
    digitalWrite(can_ss[bid], HIGH);
  }
  digitalWrite(BOOT_LED, LOW);
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
#define QUANTA 16

// 1 + propseg + 1 + phaseseg1 + 1 [@SP] + phaseseg2 + 1 == TotalTimeQuanta
static int canbaud(byte bid, unsigned bitrate)  //sets bitrate for CAN node
{
  byte q = QUANTA;
  byte propseg = pgm_read_byte_near(quantimings + ((q - 8) << 2));
  byte phaseseg1 = pgm_read_byte_near(quantimings + ((q - 8) << 2) + 1);
  byte phaseseg2 = pgm_read_byte_near(quantimings + ((q - 8) << 2) + 2);
  byte syncjump = pgm_read_byte_near(quantimings + ((q - 8) << 2) + 3);

  unsigned maxrate = (CAN_XTAL / 1000 / 2) / (propseg + phaseseg1 + phaseseg2 + 4);

  byte cnf1, cnf2, cnf3;
  cnf1 = (maxrate / bitrate) - 1;
  if (cnf1 > 63 || (cnf1 + 1) * bitrate != maxrate)
    return -1;              // overflow or inexact
  //cnf1 |= syncjump;

  cnf2 = 0x80 | phaseseg1 << 3 | propseg;
  cnf3 = phaseseg2;

  digitalWrite(can_ss[bid], LOW);
  SPI.transfer(WRITE);
  SPI.transfer(CNF3);
  SPI.transfer(cnf3);
  SPI.transfer(cnf2);
  SPI.transfer(cnf1);
  digitalWrite(can_ss[bid], HIGH);
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
void canmode(byte bid, byte mode)       //put CAN controller in one of five modes
{
  digitalWrite(can_ss[bid], LOW);
  SPI.transfer(BIT_MODIFY);
  SPI.transfer(CANCTRL);
  SPI.transfer(0xe7);
  mode &= ~7;
  if ( !bid )
    mode |= 4;
  SPI.transfer(mode);
  digitalWrite(can_ss[bid], HIGH);
}

#ifdef USEINTS
// Enable / Disable interrupt pin on message Rx
void canrxinte(byte bid, bool enable)
{
  digitalWrite(can_ss[bid], LOW);
  SPI.transfer(BIT_MODIFY);
  SPI.transfer(CANINTE);
  SPI.transfer(3);
  SPI.transfer(enable ? 3 : 0);
  digitalWrite(can_ss[bid], HIGH);
}

void intcanrx0()
{
  canread(0);
}

void intcanrx1()
{
  canread(1);
}

void intcanrx2()
{
  canread(2);
}
#endif
static byte can_reset[3];
static unsigned int canrates[8] = { 500, 250, 125, 100, 50, 20, 10 };   //, 1000

//  note 83(.3)k, 33(.3)k and others are also doable
static unsigned int curbusrate[3] = { 500, 250, 125 };

static void setcan(byte bid, byte ss, byte reset, byte rate)
{
  // set the slaveSelectPin as an output
  can_ss[bid] = ss;
  can_reset[bid] = reset;
  pinMode(ss, OUTPUT);
  pinMode(reset, OUTPUT);
  digitalWrite(reset, LOW);   /* RESET CAN CONTROLLER */
  delay(50);
  digitalWrite(reset, HIGH);
  delay(100);
  if (!canbaud(bid, rate))
    curbusrate[bid] = rate;
  else
    canbaud(bid, 500);
  canmode(bid, NORMAL);
#ifdef USEINTS
  canrxinte(bid, true);
#endif
}

void setup()
{
  Serial.begin(115200);
  pinMode(BOOT_LED, OUTPUT);
  digitalWrite(BOOT_LED, LOW);

  // SPI setup
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV2); // 8MHz
  SPI.setBitOrder(MSBFIRST);

  // setup CANs
  byte j, cb;
  for (j = 0; j < 3; j++) {
    //cb = EEPROM.read(j);
    //if (cb < sizeof(canrates))
    curbusrate[j] = 250;    //canrates[cb];
  }
#define CAN1RESET 4
#define CAN1INT 3
#define CAN1SELECT 9
  setcan(0, CAN1SELECT, CAN1RESET, curbusrate[0]);
#define CAN2RESET 12
#define CAN2INT 2
#define CAN2SELECT 10
  setcan(1, CAN2SELECT, CAN2RESET, curbusrate[1]);
#define CAN3RESET 11
#define CAN3INT 7
#define CAN3SELECT 5
  setcan(2, CAN3SELECT, CAN3RESET, curbusrate[2]);

#ifdef USEINTS
  attachInterrupt(CAN1INT, intcanrx0, LOW);
  attachInterrupt(CAN2INT, intcanrx1, LOW);
  attachInterrupt(CAN3INT, intcanrx2, LOW);
#endif

  cansend(2, 0x08880808, true, 8, (byte *) "HelloCAN");

}

#ifdef EMIT_B64
/*prog_ */ char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
// bus:2 dlc:4
// extid:1 highest:5
// for extid (hi-mid 6) (mid 6) (lo-mid 6)
// lowest:6
// data, 0, 2==, 3=, 4, 6==, 7=, 8, 10==, 11=
#endif

void printcanrx()
{
  while (canmsghead != canmsgtail) {

    byte bid, *frameinfo = canmsgbuf[canmsgtail++];

    if (canmsgtail >= CANBUFSIZ)
      canmsgtail = 0;

    bid = *frameinfo++;

    Serial.print(bid);
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
    *benc++ = b64[(bid << 4) | (frameinfo[4] & 0x0f)];

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
  static unsigned long x;
  canread(0);
  canread(1);
  canread(2);
  printcanrx();
#if 0
  if (!(x++ & 65535))
    cansend(2, 0x08880808, true, 8, (byte *) "HelloCAN");
  if (!((x ^ 32768) & 65535))
    cansend(2, 0x7E8, false, 8, (byte *) "@UU@U@@U");
#endif
}
