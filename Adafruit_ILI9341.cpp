/***************************************************
  This is our library for the Adafruit ILI9341 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_ILI9341.h"
#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

// If the SPI library has transaction support, these functions
// establish settings and protect from interference from other
// libraries.  Otherwise, they simply do nothing.
inline void Adafruit_ILI9341::spi_begin(void){
#ifdef SPI_HAS_TRANSACTION
#if defined (ARDUINO_ARCH_ARC32)
  // max speed!
  _SPI->beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
#elif defined(ESP8266)
  _SPI->beginTransaction(SPISettings(ESP8266_CLOCK, MSBFIRST, SPI_MODE0));
#elif defined(__STM32F1__)
  _SPI->beginTransaction(SPISettings(36000000, MSBFIRST, SPI_MODE0));
#else
    // max speed!
  _SPI->beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE0));
#endif
#endif
}
inline void Adafruit_ILI9341::spi_end(void){
#ifdef SPI_HAS_TRANSACTION
  _SPI->endTransaction();
#endif
}

// Constructor when using software SPI.  All output pins are configurable.
Adafruit_ILI9341::Adafruit_ILI9341(int8_t cs, int8_t dc, int8_t mosi,
				   int8_t sclk, int8_t rst, int8_t miso) : Adafruit_GFX(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT) {
  _cs   = cs;
  _dc   = dc;
  _mosi  = mosi;
  _miso = miso;
  _sclk = sclk;
  _rst  = rst;
  hwSPI = false;
}


// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
Adafruit_ILI9341::Adafruit_ILI9341(int8_t cs, int8_t dc, int8_t rst, SPIClass *SPIdev) : Adafruit_GFX(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT) {
  _cs   = cs;
  _dc   = dc;
  _rst  = rst;
  hwSPI = true;
  _mosi  = _sclk = 0;
  _SPI = SPIdev;
}

void Adafruit_ILI9341::spiwrite(uint8_t c) {

  //Serial.print("0x"); Serial.print(c, HEX); Serial.print(", ");

  if (hwSPI) {
#if defined (__AVR__)
  #ifndef SPI_HAS_TRANSACTION
    uint8_t backupSPCR = SPCR;
    SPCR = mySPCR;
  #endif
    SPDR = c;
    while(!(SPSR & _BV(SPIF)));
  #ifndef SPI_HAS_TRANSACTION
    SPCR = backupSPCR;
  #endif
#else
    _SPI->transfer(c);
#endif
  } else {
#ifndef USE_FAST_PINIO
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
      if(c & bit) {
	digitalWrite(_mosi, HIGH); 
      } else {
	digitalWrite(_mosi, LOW); 
      }
      digitalWrite(_sclk, HIGH);
      digitalWrite(_sclk, LOW);
    }
#else
    // Fast SPI bitbang swiped from LPD8806 library
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
      if(c & bit) {
	//digitalWrite(_mosi, HIGH); 
	*mosiport |=  mosipinmask;
      } else {
	//digitalWrite(_mosi, LOW); 
	*mosiport &= ~mosipinmask;
      }
      //digitalWrite(_sclk, HIGH);
      *clkport |=  clkpinmask;
      //digitalWrite(_sclk, LOW);
      *clkport &= ~clkpinmask;
    }
#endif
  }
}


void Adafruit_ILI9341::writecommand(uint8_t c) {
#if defined (USE_FAST_PINIO)
  *dcport &= ~dcpinmask;
  *csport &= ~cspinmask;
#else
  digitalWrite(_dc, LOW);
  if(!hwSPI) digitalWrite(_sclk, LOW);
  digitalWrite(_cs, LOW);
#endif

  spiwrite(c);

#if defined (USE_FAST_PINIO)
  *csport |= cspinmask;
#else
  digitalWrite(_cs, HIGH);
#endif
}


void Adafruit_ILI9341::writedata(uint8_t c) {
#if defined (USE_FAST_PINIO)
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;
#else
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);
#endif

  spiwrite(c);

#if defined (USE_FAST_PINIO)
  *csport |= cspinmask;
#else
  digitalWrite(_cs, HIGH);
#endif
} 


// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void Adafruit_ILI9341::commandList(uint8_t *addr) {

  uint8_t  numCommands, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    writecommand(pgm_read_byte(addr++)); //   Read, issue command
    numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while(numArgs--) {                   //   For each argument...
      writedata(pgm_read_byte(addr++));  //     Read, issue argument
    }

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      delay(ms);
    }
  }
}


void Adafruit_ILI9341::begin(void) {
  if (_rst) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, LOW);
  }

  pinMode(_dc, OUTPUT);
  pinMode(_cs, OUTPUT);

#if defined (USE_FAST_PINIO)
  csport    = portOutputRegister(digitalPinToPort(_cs));
  cspinmask = digitalPinToBitMask(_cs);
  dcport    = portOutputRegister(digitalPinToPort(_dc));
  dcpinmask = digitalPinToBitMask(_dc);
#endif

  if(hwSPI) { // Using hardware SPI
    _SPI->begin();

#ifndef SPI_HAS_TRANSACTION
    _SPI->setBitOrder(MSBFIRST);
    _SPI->setDataMode(SPI_MODE0);
  #if defined (_AVR__)
    _SPI->setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
    mySPCR = SPCR;
  #elif defined(ESP8266)
    _SPI->setFrequency(80000000);
  #elif defined(__STM32F1__)
    _SPI->setClockDivider(SPI_CLOCK_DIV2);
  #elif defined(TEENSYDUINO)
    _SPI->setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
  #elif defined (__arm__)
    _SPI->setClockDivider(11); // 8-ish MHz (full! speed!)
  #endif
#endif
  } else {
    pinMode(_sclk, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);

#if defined (USE_FAST_PINIO)
    clkport     = portOutputRegister(digitalPinToPort(_sclk));
    clkpinmask  = digitalPinToBitMask(_sclk);
    mosiport    = portOutputRegister(digitalPinToPort(_mosi));
    mosipinmask = digitalPinToBitMask(_mosi);
    *clkport   &= ~clkpinmask;
    *mosiport  &= ~mosipinmask;
#endif
  }

  // toggle RST low to reset
  if (_rst) {
    digitalWrite(_rst, HIGH);
    delay(5);
    digitalWrite(_rst, LOW);
    delay(20);
    digitalWrite(_rst, HIGH);
    delay(150);
  }

  /*
  uint8_t x = readcommand8(ILI9341_RDMODE);
  Serial.print("\nDisplay Power Mode: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDMADCTL);
  Serial.print("\nMADCTL Mode: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDPIXFMT);
  Serial.print("\nPixel Format: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDIMGFMT);
  Serial.print("\nImage Format: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDSELFDIAG);
  Serial.print("\nSelf Diagnostic: 0x"); Serial.println(x, HEX);
*/
  //if(cmdList) commandList(cmdList);
  
  if (hwSPI) spi_begin();
  writecommand(0xEF);
  writedata(0x03);
  writedata(0x80);
  writedata(0x02);

  writecommand(0xCF);  
  writedata(0x00); 
  writedata(0XC1); 
  writedata(0X30); 

  writecommand(0xED);  
  writedata(0x64); 
  writedata(0x03); 
  writedata(0X12); 
  writedata(0X81); 
 
  writecommand(0xE8);  
  writedata(0x85); 
  writedata(0x00); 
  writedata(0x78); 

  writecommand(0xCB);  
  writedata(0x39); 
  writedata(0x2C); 
  writedata(0x00); 
  writedata(0x34); 
  writedata(0x02); 
 
  writecommand(0xF7);  
  writedata(0x20); 

  writecommand(0xEA);  
  writedata(0x00); 
  writedata(0x00); 
 
  writecommand(ILI9341_PWCTR1);    //Power control 
  writedata(0x23);   //VRH[5:0] 
 
  writecommand(ILI9341_PWCTR2);    //Power control 
  writedata(0x10);   //SAP[2:0];BT[3:0] 
 
  writecommand(ILI9341_VMCTR1);    //VCM control 
  writedata(0x3e); //¶Ô±È¶Èµ÷½Ú
  writedata(0x28); 
  
  writecommand(ILI9341_VMCTR2);    //VCM control2 
  writedata(0x86);  //--
 
  writecommand(ILI9341_MADCTL);    // Memory Access Control 
  writedata(0x48);

  writecommand(ILI9341_PIXFMT);    
  writedata(0x55); 
  
  writecommand(ILI9341_FRMCTR1);    
  writedata(0x00);  
  writedata(0x18); 
 
  writecommand(ILI9341_DFUNCTR);    // Display Function Control 
  writedata(0x08); 
  writedata(0x82);
  writedata(0x27);  
 
  writecommand(0xF2);    // 3Gamma Function Disable 
  writedata(0x00); 
 
  writecommand(ILI9341_GAMMASET);    //Gamma curve selected 
  writedata(0x01); 
 
  writecommand(ILI9341_GMCTRP1);    //Set Gamma 
  writedata(0x0F); 
  writedata(0x31); 
  writedata(0x2B); 
  writedata(0x0C); 
  writedata(0x0E); 
  writedata(0x08); 
  writedata(0x4E); 
  writedata(0xF1); 
  writedata(0x37); 
  writedata(0x07); 
  writedata(0x10); 
  writedata(0x03); 
  writedata(0x0E); 
  writedata(0x09); 
  writedata(0x00); 
  
  writecommand(ILI9341_GMCTRN1);    //Set Gamma 
  writedata(0x00); 
  writedata(0x0E); 
  writedata(0x14); 
  writedata(0x03); 
  writedata(0x11); 
  writedata(0x07); 
  writedata(0x31); 
  writedata(0xC1); 
  writedata(0x48); 
  writedata(0x08); 
  writedata(0x0F); 
  writedata(0x0C); 
  writedata(0x31); 
  writedata(0x36); 
  writedata(0x0F); 

  writecommand(ILI9341_SLPOUT);    //Exit Sleep 
  if (hwSPI) spi_end();
  delay(120); 		
  if (hwSPI) spi_begin();
  writecommand(ILI9341_DISPON);    //Display on 
  if (hwSPI) spi_end();

}

void Adafruit_ILI9341::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
 uint16_t y1) {
  if(hwSPI) spi_begin();
  setAddrWindow_(x0, y0, x1, y1);
  if(hwSPI) spi_end();
}

void Adafruit_ILI9341::setAddrWindow_(uint16_t x0, uint16_t y0, uint16_t x1,
 uint16_t y1) {
  uint8_t buffC[] = { (uint8_t) (x0 >> 8), (uint8_t) x0, (uint8_t) (x1 >> 8), (uint8_t) x1 };
  uint8_t buffP[] = { (uint8_t) (y0 >> 8), (uint8_t) y0, (uint8_t) (y1 >> 8), (uint8_t) y1 };
  #if defined(USE_FAST_PINIO)
  *dcport &= ~dcpinmask;
  *csport &= ~cspinmask;
  #else
  digitalWrite(_dc, LOW);
  digitalWrite(_cs, LOW);
  #endif

  spiwrite(ILI9341_CASET);

  #if defined(USE_FAST_PINIO)
  *dcport |=  dcpinmask;
  #else
  digitalWrite(_dc, HIGH);
  #endif

  if(hwSPI){
  #ifdef ESP8266
    _SPI->writePattern(&buffC[0], 4, 1);
  #else
    _SPI->transfer(buffC[0]);
    _SPI->transfer(buffC[1]);
    _SPI->transfer(buffC[2]);
    _SPI->transfer(buffC[3]);
  #endif
  }
  else {
    spiwrite(buffC[0]);
    spiwrite(buffC[1]);
    spiwrite(buffC[2]);
    spiwrite(buffC[3]);
  }

  #if defined(USE_FAST_PINIO)
  *dcport &= ~dcpinmask;
  #else
  digitalWrite(_dc, LOW);
  #endif

  spiwrite(ILI9341_PASET);

  #if defined(USE_FAST_PINIO)
  *dcport |=  dcpinmask;
  #else
  digitalWrite(_dc, HIGH);
  #endif

  if(hwSPI){
  #ifdef ESP8266
    _SPI->writePattern(&buffP[0], 4, 1);
  #else
    _SPI->transfer(buffP[0]);
    _SPI->transfer(buffP[1]);
    _SPI->transfer(buffP[2]);
    _SPI->transfer(buffP[3]);
  #endif
  }
  else {
    spiwrite(buffP[0]);
    spiwrite(buffP[1]);
    spiwrite(buffP[2]);
    spiwrite(buffP[3]);
  }

  #if defined(USE_FAST_PINIO)
  *dcport &= ~dcpinmask;
  #else
  digitalWrite(_dc, LOW);
  #endif

  spiwrite(ILI9341_RAMWR);

  #if defined(USE_FAST_PINIO)
  *dcport |=  dcpinmask;
  #else
  digitalWrite(_dc, HIGH);
  #endif

  #if defined(USE_FAST_PINIO)
  *csport |= cspinmask;
  #else
  digitalWrite(_cs, HIGH);
  #endif
}

void Adafruit_ILI9341::pushColor(uint16_t color) {
  if (hwSPI) spi_begin();

#if defined(USE_FAST_PINIO)
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;
#else
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);
#endif

  spiwrite(color >> 8);
  spiwrite(color);

#if defined(USE_FAST_PINIO)
  *csport |= cspinmask;
#else
  digitalWrite(_cs, HIGH);
#endif

  if (hwSPI) spi_end();
}

void Adafruit_ILI9341::drawPixel(int16_t x, int16_t y, uint16_t color) {

  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

  if (hwSPI) spi_begin();
  setAddrWindow_(x,y,x+1,y+1);

#if defined(USE_FAST_PINIO)
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;
#else
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);
#endif

  spiwrite(color >> 8);
  spiwrite(color);

#if defined(USE_FAST_PINIO)
  *csport |= cspinmask;
#else
  digitalWrite(_cs, HIGH);
#endif

  if (hwSPI) spi_end();
}


void Adafruit_ILI9341::drawFastVLine(int16_t x, int16_t y, int16_t h,
 uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;

  if((y+h-1) >= _height) 
    h = _height-y;

  if (hwSPI) spi_begin();
  setAddrWindow_(x, y, x, y+h-1);

#if defined(USE_FAST_PINIO)
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;
#else
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);
#endif

  uint8_t colorBin[] = { (uint8_t) (color >> 8), (uint8_t) color };
#if defined (__STM32F1__)
  if(hwSPI){
    _SPI->setDataSize (SPI_CR1_DFF); // Set SPI 16bit mode
    _SPI->dmaSend(&color, h, 0);
    _SPI->setDataSize (0);
  } else
#elif defined (ESP8266)
  if(hwSPI) _SPI->writePattern(&colorBin[0], 2, h);
  else
#endif
  spiWriteBytes(&colorBin[0], 2, h);

#if defined(USE_FAST_PINIO)
  *csport |= cspinmask;
#else
  digitalWrite(_cs, HIGH);
#endif

  if (hwSPI) spi_end();
}


void Adafruit_ILI9341::drawFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  if (hwSPI) spi_begin();
  setAddrWindow_(x, y, x+w-1, y);

#if defined(USE_FAST_PINIO)
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;
#else
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);
#endif

  uint8_t colorBin[] = { (uint8_t) (color >> 8), (uint8_t) color };
#if defined (__STM32F1__)
  if(hwSPI){
    _SPI->setDataSize (SPI_CR1_DFF); // Set SPI 16bit mode
    _SPI->dmaSend(&color, w, 0);
    _SPI->setDataSize (0);
  } else
#elif defined (ESP8266)
  if(hwSPI) _SPI->writePattern(&colorBin[0], 2, w);
  else
#endif
  spiWriteBytes(&colorBin[0], 2, w);

#if defined(USE_FAST_PINIO)
  *csport |= cspinmask;
#else
  digitalWrite(_cs, HIGH);
#endif
  if (hwSPI) spi_end();
}

void Adafruit_ILI9341::fillScreen(uint16_t color) {
  fillRect(0, 0,  _width, _height, color);
}

// fill a rectangle
void Adafruit_ILI9341::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  if (hwSPI) spi_begin();
  setAddrWindow_(x, y, x+w-1, y+h-1);

#if defined(USE_FAST_PINIO)
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;
#else
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);
#endif

  uint8_t colorBin[] = { (uint8_t) (color >> 8), (uint8_t) color };
#if defined (__STM32F1__)
  if(hwSPI){
    _SPI->setDataSize (SPI_CR1_DFF); // Set spi 16bit mode
    if (w*h <= 65535)
      _SPI->dmaSend(&color, (w*h), 0);
    else {
      _SPI->dmaSend(&color, (65535), 0);
      _SPI->dmaSend(&color, ((w*h) - 65535), 0);
    }
    _SPI->setDataSize (0);
  } else
#elif defined (ESP8266)
  if(hwSPI) _SPI->writePattern(&colorBin[0], 2, w*h);
  else
#endif
  spiWriteBytes(&colorBin[0], 2, w*h);

#if defined(USE_FAST_PINIO)
  *csport |= cspinmask;
#else
  digitalWrite(_cs, HIGH);
#endif

  if (hwSPI) spi_end();
}


// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Adafruit_ILI9341::color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void Adafruit_ILI9341::setRotation(uint8_t m) {

  if (hwSPI) spi_begin();
  writecommand(ILI9341_MADCTL);
  rotation = m % 4; // can't be higher than 3
  _horzFlip=false;
  _vertFlip=false;
  switch (rotation) {
   case 0:
     writedata(MADCTL_MX | MADCTL_BGR);
     _width  = ILI9341_TFTWIDTH;
     _height = ILI9341_TFTHEIGHT;
     break;
   case 1:
     writedata(MADCTL_MV | MADCTL_BGR);
     _width  = ILI9341_TFTHEIGHT;
     _height = ILI9341_TFTWIDTH;
     break;
  case 2:
    writedata(MADCTL_MY | MADCTL_BGR);
     _width  = ILI9341_TFTWIDTH;
     _height = ILI9341_TFTHEIGHT;
    break;
   case 3:
     writedata(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
     _width  = ILI9341_TFTHEIGHT;
     _height = ILI9341_TFTWIDTH;
     break;
  }
  if (hwSPI) spi_end();
}

void Adafruit_ILI9341::invertDisplay(boolean i) {
  if (hwSPI) spi_begin();
  writecommand(i ? ILI9341_INVON : ILI9341_INVOFF);
  if (hwSPI) spi_end();
}


////////// stuff not actively being used, but kept for posterity


uint8_t Adafruit_ILI9341::spiread(void) {
  uint8_t r = 0;

  if (hwSPI) {
#if defined (__AVR__)
  #ifndef SPI_HAS_TRANSACTION
    uint8_t backupSPCR = SPCR;
    SPCR = mySPCR;
  #endif
    SPDR = 0x00;
    while(!(SPSR & _BV(SPIF)));
    r = SPDR;

  #ifndef SPI_HAS_TRANSACTION
    SPCR = backupSPCR;
  #endif
#else
    r = _SPI->transfer(0x00);
#endif

  } else {

    for (uint8_t i=0; i<8; i++) {
      digitalWrite(_sclk, LOW);
      digitalWrite(_sclk, HIGH);
      r <<= 1;
      if (digitalRead(_miso))
	r |= 0x1;
    }
  }
  //Serial.print("read: 0x"); Serial.print(r, HEX);
  
  return r;
}

 uint8_t Adafruit_ILI9341::readdata(void) {
   digitalWrite(_dc, HIGH);
   digitalWrite(_cs, LOW);
   uint8_t r = spiread();
   digitalWrite(_cs, HIGH);
   
   return r;
}
 

uint8_t Adafruit_ILI9341::readcommand8(uint8_t c, uint8_t index) {
   if (hwSPI) spi_begin();
   digitalWrite(_dc, LOW); // command
   digitalWrite(_cs, LOW);
   spiwrite(0xD9);  // woo sekret command?
   digitalWrite(_dc, HIGH); // data
   spiwrite(0x10 + index);
   digitalWrite(_cs, HIGH);

   digitalWrite(_dc, LOW);
   if(!hwSPI) digitalWrite(_sclk, LOW);
   digitalWrite(_cs, LOW);
   spiwrite(c);
 
   digitalWrite(_dc, HIGH);
   uint8_t r = spiread();
   digitalWrite(_cs, HIGH);
   if (hwSPI) spi_end();
   return r;
}

void Adafruit_ILI9341::spiWriteBytes(uint8_t * data, uint32_t size, uint32_t repeat){
  uint8_t *dataPtr=data;
  uint32_t _size=size;
  #ifdef ESP8266
  uint8_t i;
  if(hwSPI){
    if(size <= 64) _SPI->writePattern(data, size, repeat);
    else while(repeat--){
      dataPtr=data;
      //Align address to 32 bits
      for(i = 0; i < ((uint32_t) dataPtr & 3); i++) _SPI->transfer((reinterpret_cast<uint8_t *>(data))[i]);
      if(size >= i) _SPI->writeBytes(dataPtr + i, size - i);
    }
  }
  else
  #elif defined (__STM32F1__)
  uint16 temp;
  uint16 *ptr=(uint16 *)data;
  if(hwSPI){
      //Setup DMA to send the same word over and over for repeat times
      if(size==2 && repeat > 8){
        temp=(ptr[0] >> 8) | (ptr[0] << 8);
        _size=repeat;
        _SPI->setDataSize (SPI_CR1_DFF);
        while(_size > 65535){
          _SPI->dmaSend(&temp, 65535, 0);
          _size-=65535;
        }
        _SPI->dmaSend(&temp, _size, 0);
        _SPI->setDataSize (0);
      //Setup DMA to send the same byte over and over for repeat times
      } else if(size==1 && repeat > 16){
        _size=repeat;
        while(_size > 65535){
          _SPI->dmaSend(data, 65535, 0);
          _size-=65535;
        }
        _SPI->dmaSend(data, _size, 0);
      //Don't use DMA for very small transfers, because setting it up takes longer than just writing the bytes out without it
      } else if(size <= 16){
        while(repeat--){
          dataPtr=data;
          _size=size;
          while(_size--) {
           _SPI->transfer(*dataPtr);
           dataPtr++;
          }
        }
      } else
      //Finally, use DMA for larger transfers
      while(repeat--){
        dataPtr=data;
        _size=size;
        while(_size > 65535){
          _SPI->dmaSend(dataPtr, 65535, 1);
          dataPtr+=65535;
          _size-=65535;
        }
        _SPI->dmaSend(dataPtr, _size, 1);
      }

  } else
  #endif
  while(repeat--){
    dataPtr=data;
    _size=size;
    while(_size--) {
      spiwrite(*dataPtr);
      dataPtr++;
    }
  }
}


void Adafruit_ILI9341::flipDisplay(bool vertical, bool horizontal){
	// Just rotate the display
	if(vertical && horizontal) Adafruit_ILI9341::setRotation(rotation ^ 2);
	else if(!vertical && !horizontal) Adafruit_ILI9341::setRotation(rotation);
	else {
		_vertFlip=vertical;
		_horzFlip=horizontal;
		if(hwSPI) spi_begin();
	        uint8_t flags=MADCTL_BGR;
		switch(rotation){
			case 0:
				if(vertical) flags |= MADCTL_MX | MADCTL_MY;
				writecommand(ILI9341_MADCTL);
				writedata(flags);
				break;
			case 1:
				if(horizontal) flags |= MADCTL_MY | MADCTL_MV;
				if(vertical) flags |= MADCTL_MV | MADCTL_MX;
				writecommand(ILI9341_MADCTL);
				writedata(flags);
				break;
			case 2:
				if(horizontal) flags |= MADCTL_MX | MADCTL_MY;
				writecommand(ILI9341_MADCTL);
				writedata(flags);
				break;
			case 3:
				if(vertical) flags |= MADCTL_MY | MADCTL_MV;
				if(horizontal) flags |= MADCTL_MV | MADCTL_MX;
				writecommand(ILI9341_MADCTL);
				writedata(flags);
				break;
		}
		if(hwSPI) spi_end();
	}
}


void Adafruit_ILI9341::flipVertical(){
if(_vertFlip) flipDisplay(false, _horzFlip);
else flipDisplay(true, _horzFlip);
}

void Adafruit_ILI9341::flipHorizontal(){
if(_horzFlip) flipDisplay(_vertFlip, false);
else flipDisplay(_vertFlip, true);
}

/*
* Draw lines faster by calculating straight sections and drawing them with fastVline and fastHline.
*/
void Adafruit_ILI9341::drawLine(int16_t x0, int16_t y0,int16_t x1, int16_t y1, uint16_t color)
{
	if ((y0 < 0 && y1 <0) || (y0 > _height && y1 > _height)) return;
	if ((x0 < 0 && x1 <0) || (x0 > _width && x1 > _width)) return;
	if (x0 < 0) x0 = 0;
	if (x1 < 0) x1 = 0;
	if (y0 < 0) y0 = 0;
	if (y1 < 0) y1 = 0;

	if (y0 == y1) {
		if (x1 > x0) {
			drawFastHLine(x0, y0, x1 - x0 + 1, color);
		}
		else if (x1 < x0) {
			drawFastHLine(x1, y0, x0 - x1 + 1, color);
		}
		else {
			drawPixel(x0, y0, color);
		}
		return;
	}
	else if (x0 == x1) {
		if (y1 > y0) {
			drawFastVLine(x0, y0, y1 - y0 + 1, color);
		}
		else {
			drawFastVLine(x0, y1, y0 - y1 + 1, color);
		}
		return;
	}

	bool steep = abs(y1 - y0) > abs(x1 - x0);
	uint16_t temp=0;
	if (steep) {
		temp=x0;
		x0 = y0;
		y0 = temp;
		temp=x1;
		x1 = y1;
		y1 = temp;
	}
	if (x0 > x1) {
		temp=x0;
		x0 = x1;
		x1 = temp;
		temp=y0;
		y0 = y1;
		y1 = temp;
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
	}
	else {
		ystep = -1;
	}

	int16_t xbegin = x0;
	if (steep) {
		for (; x0 <= x1; x0++) {
			err -= dy;
			if (err < 0) {
				int16_t len = x0 - xbegin;
				if (len) {
					drawFastVLine (y0, xbegin, len + 1, color);
				}
				else {
					drawPixel(y0, x0, color);
				}
				xbegin = x0 + 1;
				y0 += ystep;
				err += dx;
			}
		}
		if (x0 > xbegin + 1) {
			drawFastVLine(y0, xbegin, x0 - xbegin, color);
		}

	}
	else {
		for (; x0 <= x1; x0++) {
			err -= dy;
			if (err < 0) {
				int16_t len = x0 - xbegin;
				if (len) {
					drawFastHLine(xbegin, y0, len + 1, color);
				}
				else {
					drawPixel(x0, y0, color);
				}
				xbegin = x0 + 1;
				y0 += ystep;
				err += dx;
			}
		}
		if (x0 > xbegin + 1) {
			drawFastHLine(xbegin, y0, x0 - xbegin, color);
		}
	}
}

void Adafruit_ILI9341::pushColors(uint16_t* buf, size_t n, bool littleEndian) {
  if (hwSPI) spi_begin();

#if defined(USE_FAST_PINIO)
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;
#else
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);
#endif
  if(hwSPI){
#ifdef ESP8266
    if(!littleEndian) spiWriteBytes((uint8_t *)buf, n << 1);
    else for(size_t i=0; i<n; i++) _SPI->write16(buf[i], true);
#else
    if(littleEndian)
      for(size_t i=0; i<n; i++) { spiwrite(buf[i] >> 8); spiwrite(buf[i]); }
    else spiWriteBytes((uint8_t *)buf, n << 1);
#endif
  } else {
      if(littleEndian)
        for(size_t i=0; i<n; i++) { spiwrite(buf[i] >> 8); spiwrite(buf[i]); }
      else spiWriteBytes((uint8_t *)buf, n << 1);
    }
#if defined(USE_FAST_PINIO)
  *csport |= cspinmask;
#else
  digitalWrite(_cs, HIGH);
#endif

  if (hwSPI) spi_end();
}

//This function doesn't actually move pixels themselves anywhere, it just changes
//the internal address the display reads the pixels to be displayed on the TFT from
//THIS IS NOT A TRADITIONAL SCROLL
void Adafruit_ILI9341::hardwareVScroll(uint8_t scrollDirection, uint16_t pixels)
{
  if(pixels < 1) return;
  if(pixels > 319) pixels %= 320;
  switch(scrollDirection){
  case DISPLAY_SCROLL_DOWN:
    if(_vscroll < pixels){
      pixels-=_vscroll;
      _vscroll=320 - pixels;
    } else _vscroll-=pixels;
    break;
  case DISPLAY_SCROLL_UP:
    if((_vscroll + pixels) > 319){
      _vscroll=_vscroll + pixels - 320;
    } else _vscroll+=pixels;
    break;
  default:
    return;
  }
  if (hwSPI) spi_begin();
  writecommand(ILI9341_SCRLSA);
  writedata(_vscroll >> 8);
  writedata(_vscroll);
  if (hwSPI) spi_end();
}

//This is a traditional scroll in the sense that it actually moves the pixels themselves around
void Adafruit_ILI9341::scroll(uint8_t scrollDirection, uint16_t pixels, uint16_t fillColor, bool doFill)
{
//TO BE DONE
}

void Adafruit_ILI9341::powerSaving(bool enable)
{
  if (hwSPI) spi_begin();
  writecommand(enable ? ILI9341_SLPIN : ILI9341_SLPOUT);
  if (hwSPI) spi_end();
  delay(120);
}

/*

 uint16_t Adafruit_ILI9341::readcommand16(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 uint16_t r = spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 uint32_t Adafruit_ILI9341::readcommand32(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 
 dummyclock();
 dummyclock();
 
 uint32_t r = spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 */
