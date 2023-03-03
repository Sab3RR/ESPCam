#pragma once
#include "Arduino.h"

struct Dot{
    uint32_t x;
    uint32_t y;
    uint32_t w;
    uint32_t h;
};

extern "C" struct RGB {
    uint8_t R;
    uint8_t G;
    uint8_t B;
};

struct Bitmap{
    bool *bitmap;
    const size_t W;
    const size_t H;
    const size_t L;


    Bitmap(size_t w, size_t h, size_t s) : W(w), H(h), L(s){
        // Serial.printf("in bitmap s = %ui \n", s);
        bitmap = (bool*)ps_malloc(s * sizeof(bool));
        if (!bitmap)
        {
            // Serial.printf("bad malloc  \n");
            ESP.restart();
        }
        // Serial.printf("malloc bitmap \n");
        for (int i = 0; i < s; ++i)
            bitmap[i] = false;
        // Serial.printf("init bitmap \n");
    }
    ~Bitmap(){
        free(bitmap);
    }
    bool getCell(uint32_t x, uint32_t y){
        return bitmap[(y * W) + x - 1];
    }
    void setCell(uint32_t x, uint32_t y, bool val){
        bitmap[(y * W) + x - 1] = val;
    }

};

struct Vector2u{
    uint32_t x;
    uint32_t y;

//     static void* operator new(std::size_t count)
//     {
//         // std::cout << "custom new for size " << count << '\n';
//         Serial.printf("ps_malloc \n");
//         return ps_malloc(count);
//     }
};
