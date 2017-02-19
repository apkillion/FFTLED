// FFT-based audio visualizer for Adafruit Circuit Playground: uses the
// built-in mic on A4, 10x NeoPixels for display.  Built on the ELM-Chan
// FFT library for AVR microcontrollers.

// The fast Fourier transform (FFT) algorithm converts a signal from the
// time domain to the frequency domain -- e.g. turning a sampled audio
// signal into a visualization of frequencies and magnitudes -- an EQ meter.

// The FFT algorithm itself is handled in the Circuit Playground library;
// the code here is mostly for converting that function's output into
// animation.  In most AV gear it's usually done with bargraph displays;
// with a 1D output (the 10 NeoPixels) we need to get creative with color
// and brightness...it won't look great in every situation (seems to work
// best with LOUD music), but it's colorful and fun to look at.  So this
// code is mostly a bunch of tables and weird fixed-point (integer) math
// that probably doesn't make much sense even with all these comments.

#include <Adafruit_CircuitPlayground.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>

// GLOBAL STUFF ------------------------------------------------------------

// Displaying EQ meter output straight from the FFT may be 'correct,' but
// isn't always visually interesting (most bins spend most time near zero).
// Dynamic level adjustment narrows in on a range of values so there's
// always something going on.  The upper and lower range are based on recent
// audio history, and on a per-bin basis (some may be more active than
// others, so this keeps one or two "loud" bins from spoiling the rest.

#define BINS   10          // FFT output is filtered down to this many bins
#define FRAMES 4           // This many FFT cycles are averaged for leveling
uint8_t lvl[FRAMES][BINS], // Bin levels for the prior #FRAMES frames
        avgLo[BINS],       // Pseudo rolling averages for bins -- lower and
        avgHi[BINS],       // upper limits -- for dynamic level adjustment.
        avgLvl[BINS],      //rolling avg for color 
        frameIdx = 0;      // Counter for lvl storage

// CALIBRATION CONSTANTS ---------------------------------------------------

const uint8_t PROGMEM

//sets the global LED colors not yet implemented made LEDs behave weird
//edit lines 83-85 to change color
//  c0[] = {191,185,190}, //gray
//  c1[] = {250,191,255}, //pink pastel
//  c2[] = {255,255,255}, //white
//  c3[] = {179,207,242}, //blue pastel
//  c4[] = {178,232,167}, //green pastel
//  c5[] = {23,107,4}, //forest green
//  c6[] = {107,40,4}, //brown
//  c7[] = {165,31,31}, // reddish brown
//  c8[] = {198,112,37}, // dark orange
//  c9[] = {163,173,184}, //pale blue
  

  // Low-level noise initially subtracted from each of 32 FFT bins
  noise[]    = { 4,3,3,3, 2,2,2,2,
                 2,2,2,2, 1,1,1,1,
                 1,1,1,1, 1,1,1,1,
                 1,1,1,1, 1,1,1,1 },
  // FFT bins (32) are then filtered down to 10 output bins (to match the
  // number of NeoPixels on Circuit Playground).  10 arrays here, one per
  // output bin.  First element of each is the number of input bins to
  // merge, second element is index of first merged bin, remaining values
  // are scaling weights as each input bin is merged into output.  The
  // merging also "de-linearizes" the FFT output, so it's closer to a
  // logarithmic scale with octaves evenly-ish spaced, music looks better.
  bin8data[] = { 1, 2, 147 },
  bin9data[] = { 2, 2, 89, 14 },
  bin2data[] = { 2, 3, 89, 14 },
  bin3data[] = { 4, 3, 15, 181, 58, 3 },
  bin4data[] = { 4, 4, 15, 181, 58, 3 },
  bin5data[] = { 6, 5, 6, 89, 185, 85, 14, 2 },
  bin6data[] = { 7, 7, 5, 60, 173, 147, 49, 9, 1 },
  bin7data[] = { 10, 8, 3, 23, 89, 170, 176, 109, 45, 14, 4, 1 },
  bin0data[] = { 13, 11, 2, 12, 45, 106, 167, 184, 147, 89, 43, 18, 6, 2, 1 },
  bin1data[] = { 18, 14, 2, 6, 19, 46, 89, 138, 175, 185, 165, 127, 85, 51, 27, 14, 7, 3, 2, 1 },
  // Pointers to 10 bin arrays, because PROGMEM arrays-of-arrays are weird:
  * const binData[] = { bin0data, bin1data, bin2data, bin3data, bin4data,
                        bin5data, bin6data, bin7data, bin8data, bin9data },
  // R,G,B values for color wheel covering 10 NeoPixels:
  reds[]   = { 73, 129, 169, 97, 133, 185, 143, 213, 219, 189 },
  greens[] = { 56, 108, 161, 51, 97, 156, 59, 117, 202, 208 },
  blues[]  = { 41, 91, 140, 24, 35, 107, 27, 0, 105, 156 },
  gamma8[] = { // Gamma correction improves the appearance of midrange colors
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3,
    3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6,
    6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9,
    10, 10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14,
    14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20,
    20, 21, 21, 22, 22, 23, 24, 24, 25, 25, 26, 27,
    27, 28, 29, 29, 30, 31, 31, 32, 33, 34, 34, 35,
    36, 37, 38, 38, 39, 40, 41, 42, 42, 43, 44, 45,
    46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57,
    58, 59, 60, 61, 62, 63, 64, 65, 66, 68, 69, 70,
    71, 72, 73, 75, 76, 77, 78, 80, 81, 82, 84, 85,
    86, 88, 89, 90, 92, 93, 94, 96, 97, 99, 100, 102,
    103, 105, 106, 108, 109, 111, 112, 114, 115, 117, 119, 120,
    122, 124, 125, 127, 129, 130, 132, 134, 136, 137, 139, 141,
    143, 145, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164,
    166, 168, 170, 172, 174, 176, 178, 180, 182, 184, 186, 188,
    191, 193, 195, 197, 199, 202, 204, 206, 209, 211, 213, 215,
    218, 220, 223, 225, 227, 230, 232, 235, 237, 240, 242, 245,
    247, 250, 252, 255 };
const uint16_t PROGMEM
  // Scaling values applied to each FFT bin (32) after noise subtraction
  // but prior to merging/filtering.  When multiplied by these values,
  // then divided by 256, these tend to produce outputs in the 0-255
  // range (VERY VERY "ISH") at normal listening levels.  These were
  // determined empirically by throwing lots of sample audio at it.
  binMul[] = { 405, 508, 486, 544, 533, 487, 519, 410,
               481, 413, 419, 410, 397, 424, 412, 411,
               511, 591, 588, 577, 554, 529, 524, 570,
               546, 559, 511, 552, 439, 488, 483, 547, },
  // Sums of bin weights for bin-merging tables above.
  binDiv[]   = { 147, 103, 103, 257, 257, 381, 444, 634, 822, 1142 };

// makes code use pixel strip (num of pixels, pin number, other variable dont touch)
  Adafruit_NeoPixel pixels = Adafruit_NeoPixel( 12, 6, NEO_GRB + NEO_KHZ800);

// SETUP FUNCTION - runs once ----------------------------------------------

void setup() {
  CircuitPlayground.begin();
  pixels.begin();
  pixels.setBrightness(255);
  pixels.show();
  //CircuitPlayground.clearPixels();
 
  // Initialize rolling average ranges
  uint8_t i;
  for(i=0; i<BINS; i++) {
    avgLo[i] = 0;
    avgHi[i] = 255;
    avgLvl[i] = 0;
  }
  for(i=0; i<FRAMES; i++) {
    memset(&lvl[i], 127, sizeof(lvl[i]));
  }
}

// LOOP FUNCTION - runs over and over - does animation ---------------------

void loop() {
  uint16_t spectrum[32]; // FFT spectrum output buffer

  CircuitPlayground.mic.fft(spectrum);

  // spectrum[] is now raw FFT output, 32 bins.

  // Remove noise and apply EQ levels
  uint8_t  i, N;
  uint16_t S;
  for(i=0; i<32; i++) {
    N = pgm_read_byte(&noise[i]);
    if(spectrum[i] > N) { // Above noise threshold: scale & clip
      S           = ((spectrum[i] - N) *
                     (uint32_t)pgm_read_word(&binMul[i])) >> 8;
      spectrum[i] = (S < 255) ? S : 255;
    } else { // Below noise threshold: clip
      spectrum[i] = 0;
    }
  }
  // spectrum[] is now noise-filtered, scaled & clipped
  // FFT output, in range 0-255, still 32 bins.

  // Filter spectrum[] from 32 elements down to 10,
  // make pretty colors out of it:

  uint16_t sum, level;
  uint8_t  j, minLvl, maxLvl, nBins, binNum, *data;

  for(i=0; i<BINS; i++) { // For each output bin (and each pixel)...
    data   = (uint8_t *)pgm_read_word(&binData[i]);
    nBins  = pgm_read_byte(&data[0]); // Number of input bins to merge
    binNum = pgm_read_byte(&data[1]); // Index of first input bin
    data  += 2;
    for(sum=0, j=0; j<nBins; j++) {
      sum += spectrum[binNum++] * pgm_read_byte(&data[j]); // Total
    }
    sum /= pgm_read_word(&binDiv[i]);                      // Average
    lvl[frameIdx][i] = sum;      // Save for rolling averages
    minLvl = maxLvl = lvl[0][i]; // Get min and max range for bin
    for(j=1; j<FRAMES; j++) {    // from prior stored frames
      if(lvl[j][i] < minLvl)      minLvl = lvl[j][i];
      else if(lvl[j][i] > maxLvl) maxLvl = lvl[j][i];
    }

    // minLvl and maxLvl indicate the extents of the FFT output for this
    // bin over the past few frames, used for vertically scaling the output
    // graph (so it looks interesting regardless of volume level).  If too
    // close together though (e.g. at very low volume levels) the graph
    // becomes super coarse and 'jumpy'...so keep some minimum distance
    // between them (also lets the graph go to zero when no sound playing):
    if((maxLvl - minLvl) < 23) {
      maxLvl = (minLvl < (255-23)) ? minLvl + 23 : 255;
    }
    avgLo[i] = (avgLo[i] * 7 + minLvl) / 8; // Dampen min/max levels
    avgHi[i] = (maxLvl >= avgHi[i]) ?       // (fake rolling averages)
      (avgHi[i] *  15 + maxLvl) /  16 :       // Fast rise
      (avgHi[i] * 5500000 + maxLvl) / 5500001;        //  Slower decay

    // Second fixed-point scale then 'stretches' each bin based on
    // dynamic min/max levels to 0-256 range:
    level = 1 + ((sum <= avgLo[i]) ? 0 :
                 256L * (sum - avgLo[i]) / (long)(avgHi[i] - avgLo[i]));
    // Clip output and convert to color:
    if(level <= 255) {
      if(level >= avgLvl[i]){
      avgLvl[i] = (avgLvl[i] * 5 + level)/ (6); //weighted lvl average rise
      level = avgLvl[i];
      uint8_t r = (pgm_read_byte(&reds[i])   * level) >> 8,
              g = (pgm_read_byte(&greens[i]) * level) >> 8,
              b = (pgm_read_byte(&blues[i])  * level) >> 8;
      pixels.setPixelColor(i,
        pgm_read_byte(&gamma8[r]),
        pgm_read_byte(&gamma8[g]),
        pgm_read_byte(&gamma8[b]));

      pixels.setPixelColor(i+10,
        pgm_read_byte(&gamma8[r]),
        pgm_read_byte(&gamma8[g]),
        pgm_read_byte(&gamma8[b]));
      }
      else{
        avgLvl[i] = (avgLvl[i] * 20 + level)/ (21); //weighted lvl average fall
      level = avgLvl[i];
      uint8_t r = (pgm_read_byte(&reds[i])   * level) >> 8,
              g = (pgm_read_byte(&greens[i]) * level) >> 8,
              b = (pgm_read_byte(&blues[i])  * level) >> 8;
      pixels.setPixelColor(i,
        pgm_read_byte(&gamma8[r]),
        pgm_read_byte(&gamma8[g]),
        pgm_read_byte(&gamma8[b]));

       pixels.setPixelColor(i+10,
        pgm_read_byte(&gamma8[r]),
        pgm_read_byte(&gamma8[g]),
        pgm_read_byte(&gamma8[b]));
       
      }
      
    } else { // level = 256, show full pixel OONTZ OONTZ
      avgLvl[i] = (avgLvl[i] * 5 + level)/ (6); //weighted lvl average 
       uint8_t r = (pgm_read_byte(&reds[i])   * 255) >> 8,
              g = (pgm_read_byte(&greens[i]) * 255) >> 8,
              b = (pgm_read_byte(&blues[i])  * 255) >> 8;
      pixels.setPixelColor(i,
        pgm_read_byte(&gamma8[r]),
        pgm_read_byte(&gamma8[g]),
        pgm_read_byte(&gamma8[b]));
      pixels.setPixelColor(i+10,
        pgm_read_byte(&gamma8[r]),
        pgm_read_byte(&gamma8[g]),
        pgm_read_byte(&gamma8[b]));
    }
  }
  pixels.show();
  delay(30);
  Serial.flush();
  

  if(++frameIdx >= FRAMES) frameIdx = 0;
}
