#ifndef IMAGES_H
#define IMAGES_H

#include <stdint.h>

struct Image {
    uint16_t width;
    uint16_t height;
    uint8_t* data;
};

extern const struct Image image_userlogo;
extern const struct Image image_mainlogo;

#endif /* IMAGES_H */
