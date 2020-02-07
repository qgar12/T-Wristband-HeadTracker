#ifndef tft_helper_h
#define tft_helper_h

#include <TFT_eSPI.h> 

// number of lines on tft
#define TFT_NB_OF_LINES 5

// macros
#define DCLEAR() {tftClear();}
//#define DPRINT(...) { char b = tfdGetBuffer(); snprintf(b, sizeof(b), __VA_ARGS__); tftPrint(b); }
#define DPRINT(...) { char* b = tfdGetBuffer(); snprintf(tftBuffer[tftCurrentLine], sizeof(tftBuffer[tftCurrentLine]), __VA_ARGS__); tftPrint(tftBuffer[tftCurrentLine]); }
#define DWAIT() {delay(2000);}

// tft mus be injected before usage of this helper
TFT_eSPI* pTft = NULL;

int tftCurrentLine = 0;
char tftBuffer[TFT_NB_OF_LINES+1][80]; // reserve one extra line for scrolling

void tftSet(TFT_eSPI tft) {
    pTft = &tft;
}

void tftClear() {
    pTft->setTextColor(TFT_GREEN, TFT_BLACK);
    pTft->fillScreen(TFT_BLACK);
    pTft->setTextDatum(TL_DATUM);
    tftCurrentLine=0;
 }

void tftPrintLine(const char *buf, const int lineNb) {
  pTft->drawString(buf, 0, lineNb * 16);
}

void tftPrint(const char *buf) {
  if (tftCurrentLine >= TFT_NB_OF_LINES) {
    // scroll & print
    tftClear();
    for (int ix=0; ix < TFT_NB_OF_LINES; ix++) {
      strncpy(tftBuffer[ix], tftBuffer[ix+1], sizeof(tftBuffer[ix]));
      tftPrintLine(tftBuffer[ix], ix);
    } 
  }
  else {
    // print
    tftPrintLine(tftBuffer[tftCurrentLine], tftCurrentLine);
    tftCurrentLine +=1;
  }
  
}

char* tfdGetBuffer() {
  return tftBuffer[tftCurrentLine];
}

#endif