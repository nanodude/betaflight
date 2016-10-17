#ifndef SPECTROGRAPH_H
#define SPECTROGRAPH_H


#define SPEC_FFT_LENGTH 1024

void spectrographInit();
void spectrographMain();
void spectrographDraw(uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, uint16_t max_freq);



#endif /* SPECTROGRAPH_H */

