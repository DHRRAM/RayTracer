#pragma once

#include "renderer.h"

inline void SetRenderImagePixel(RenderImage& image, int x, int y, const Color24& color) {
    int width = image.GetWidth();
    int height = image.GetHeight();
    if (x >= 0 && x < width && y >= 0 && y < height) {
        Color24* pixels = image.GetPixels();
        pixels[y * width + x] = color;
    }
}