// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "camera_index.h"
#include "Arduino.h"

#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"

#include <vector>
#include <list>
#include <memory>

#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7

#define FACE_COLOR_WHITE  0x00FFFFFF
#define FACE_COLOR_BLACK  0x00000000
#define FACE_COLOR_RED    0x000000FF
#define FACE_COLOR_GREEN  0x0000FF00
#define FACE_COLOR_BLUE   0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN   (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)

typedef struct {
        size_t size; //number of values used for filtering
        size_t index; //current value index
        size_t count; //value count
        int sum;
        int * values; //array to be filled with values
} ra_filter_t;

typedef struct {
        httpd_req_t *req;
        size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static ra_filter_t ra_filter;
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static mtmn_config_t mtmn_config = {0};
static int8_t detection_enabled = 0;
static int8_t recognition_enabled = 0;
static int8_t is_enrolling = 0;
static face_id_list id_list = {0};

static ra_filter_t * ra_filter_init(ra_filter_t * filter, size_t sample_size){
    memset(filter, 0, sizeof(ra_filter_t));

    filter->values = (int *)malloc(sample_size * sizeof(int));
    if(!filter->values){
        return NULL;
    }
    memset(filter->values, 0, sample_size * sizeof(int));

    filter->size = sample_size;
    return filter;
}

static int ra_filter_run(ra_filter_t * filter, int value){
    if(!filter->values){
        return value;
    }
    filter->sum -= filter->values[filter->index];
    filter->values[filter->index] = value;
    filter->sum += filter->values[filter->index];
    filter->index++;
    filter->index = filter->index % filter->size;
    if (filter->count < filter->size) {
        filter->count++;
    }
    return filter->sum / filter->count;
}

static void rgb_print(dl_matrix3du_t *image_matrix, uint32_t color, const char * str){
    fb_data_t fb;
    fb.width = image_matrix->w;
    fb.height = image_matrix->h;
    fb.data = image_matrix->item;
    fb.bytes_per_pixel = 3;
    fb.format = FB_BGR888;
    fb_gfx_print(&fb, (fb.width - (strlen(str) * 14)) / 2, 10, color, str);
}

static int rgb_printf(dl_matrix3du_t *image_matrix, uint32_t color, const char *format, ...){
    char loc_buf[64];
    char * temp = loc_buf;
    int len;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
    va_end(copy);
    if(len >= sizeof(loc_buf)){
        temp = (char*)malloc(len+1);
        if(temp == NULL) {
            return 0;
        }
    }
    vsnprintf(temp, len+1, format, arg);
    va_end(arg);
    rgb_print(image_matrix, color, temp);
    if(len > 64){
        free(temp);
    }
    return len;
}

static void draw_face_boxes(dl_matrix3du_t *image_matrix, box_array_t *boxes, int face_id){
    int x, y, w, h, i;
    uint32_t color = FACE_COLOR_YELLOW;
    if(face_id < 0){
        color = FACE_COLOR_RED;
    } else if(face_id > 0){
        color = FACE_COLOR_GREEN;
    }
    fb_data_t fb;
    fb.width = image_matrix->w;
    fb.height = image_matrix->h;
    fb.data = image_matrix->item;
    fb.bytes_per_pixel = 3;
    fb.format = FB_BGR888;
    for (i = 0; i < boxes->len; i++){
        // rectangle box
        x = (int)boxes->box[i].box_p[0];
        y = (int)boxes->box[i].box_p[1];
        w = (int)boxes->box[i].box_p[2] - x + 1;
        h = (int)boxes->box[i].box_p[3] - y + 1;
        fb_gfx_drawFastHLine(&fb, x, y, w, color);
        fb_gfx_drawFastHLine(&fb, x, y+h-1, w, color);
        fb_gfx_drawFastVLine(&fb, x, y, h, color);
        fb_gfx_drawFastVLine(&fb, x+w-1, y, h, color);
#if 0
        // landmark
        int x0, y0, j;
        for (j = 0; j < 10; j+=2) {
            x0 = (int)boxes->landmark[i].landmark_p[j];
            y0 = (int)boxes->landmark[i].landmark_p[j+1];
            fb_gfx_fillRect(&fb, x0, y0, 3, 3, color);
        }
#endif
    }
}

static int run_face_recognition(dl_matrix3du_t *image_matrix, box_array_t *net_boxes){
    dl_matrix3du_t *aligned_face = NULL;
    int matched_id = 0;

    aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
    if(!aligned_face){
        Serial.println("Could not allocate face recognition buffer");
        return matched_id;
    }
    if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK){
        if (is_enrolling == 1){
            int8_t left_sample_face = enroll_face(&id_list, aligned_face);

            if(left_sample_face == (ENROLL_CONFIRM_TIMES - 1)){
                Serial.printf("Enrolling Face ID: %d\n", id_list.tail);
            }
            Serial.printf("Enrolling Face ID: %d sample %d\n", id_list.tail, ENROLL_CONFIRM_TIMES - left_sample_face);
            rgb_printf(image_matrix, FACE_COLOR_CYAN, "ID[%u] Sample[%u]", id_list.tail, ENROLL_CONFIRM_TIMES - left_sample_face);
            if (left_sample_face == 0){
                is_enrolling = 0;
                Serial.printf("Enrolled Face ID: %d\n", id_list.tail);
            }
        } else {
            matched_id = recognize_face(&id_list, aligned_face);
            if (matched_id >= 0) {
                Serial.printf("Match Face ID: %u\n", matched_id);
                rgb_printf(image_matrix, FACE_COLOR_GREEN, "Hello Subject %u", matched_id);
            } else {
                Serial.println("No Match Found");
                rgb_print(image_matrix, FACE_COLOR_RED, "Intruder Alert!");
                matched_id = -1;
            }
        }
    } else {
        Serial.println("Face Not Aligned");
        //rgb_print(image_matrix, FACE_COLOR_YELLOW, "Human Detected");
    }

    dl_matrix3du_free(aligned_face);
    return matched_id;
}

static size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len){
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if(!index){
        j->len = 0;
    }
    if(httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK){
        return 0;
    }
    j->len += len;
    return len;
}

static esp_err_t capture_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    int64_t fr_start = esp_timer_get_time();

    fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    size_t out_len, out_width, out_height;
    uint8_t * out_buf;
    bool s;
    bool detected = false;
    int face_id = 0;
    Serial.printf("format %i", (int)fb->format);
    if(!detection_enabled || fb->width > 400){
        size_t fb_len = 0;
        if(fb->format == PIXFORMAT_JPEG){
            fb_len = fb->len;
            res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
        } else {
            jpg_chunking_t jchunk = {req, 0};
            res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk)?ESP_OK:ESP_FAIL;
            httpd_resp_send_chunk(req, NULL, 0);
            fb_len = jchunk.len;
        }
        
        
        esp_camera_fb_return(fb);
        int64_t fr_end = esp_timer_get_time();
        Serial.printf("JPG: %uB %ums\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start)/1000));
        return res;
    }

    dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
    if (!image_matrix) {
        esp_camera_fb_return(fb);
        Serial.println("dl_matrix3du_alloc failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    out_buf = image_matrix->item;
    out_len = fb->width * fb->height * 3;
    out_width = fb->width;
    out_height = fb->height;

    s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
    esp_camera_fb_return(fb);
    if(!s){
        dl_matrix3du_free(image_matrix);
        Serial.println("to rgb888 failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);

    if (net_boxes){
        detected = true;
        if(recognition_enabled){
            face_id = run_face_recognition(image_matrix, net_boxes);
        }
        draw_face_boxes(image_matrix, net_boxes, face_id);
        free(net_boxes->score);
        free(net_boxes->box);
        free(net_boxes->landmark);
        free(net_boxes);
    }

    jpg_chunking_t jchunk = {req, 0};
    s = fmt2jpg_cb(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, jpg_encode_stream, &jchunk);
    dl_matrix3du_free(image_matrix);
    if(!s){
        Serial.println("JPEG compression failed");
        return ESP_FAIL;
    }

    int64_t fr_end = esp_timer_get_time();
    Serial.printf("FACE: %uB %ums %s%d\n", (uint32_t)(jchunk.len), (uint32_t)((fr_end - fr_start)/1000), detected?"DETECTED ":"", face_id);
    return res;
}

extern "C" struct RGB {
    uint8_t R;
    uint8_t G;
    uint8_t B;
};


struct Dot{
    uint32_t x;
    uint32_t y;
    uint32_t w;
    uint32_t h;
};

struct Map{
    uint8_t *map;
    const size_t W;
    const size_t H;
    const size_t L;
    const size_t realL;

    Map(size_t w, size_t h, size_t l): W(w), H(h), L(l), realL(l * 3){

    }

    RGB * getMap(uint32_t x, uint32_t y){
        return (RGB*)&(map[((y * W) + (x - 1)) * 3]);
    }

    void setMap(uint32_t x, uint32_t y, RGB rgb){
        ((RGB*)&(map[((y * W) + (x - 1)) * 3]))->R = rgb.R;
        ((RGB*)&(map[((y * W) + (x - 1)) * 3]))->G = rgb.G;
        ((RGB*)&(map[((y * W) + (x - 1)) * 3]))->B = rgb.B;
    }
};
template<typename _Tp>
class PSRam_Allocator : public std::allocator<_Tp>{
public:
    typedef size_t     size_type;
      typedef ptrdiff_t  difference_type;
      typedef _Tp*       pointer;
      typedef const _Tp* const_pointer;
      typedef _Tp&       reference;
      typedef const _Tp& const_reference;
      typedef _Tp        value_type;
    

    PSRam_Allocator() throw() : std::allocator<_Tp>() { }

      PSRam_Allocator(const PSRam_Allocator& __a) throw()
      : std::allocator<_Tp>(__a) { }

      template<typename _Tp1>
        PSRam_Allocator(const PSRam_Allocator<_Tp1>& __a) throw(): std::allocator<_Tp>(__a) { }

      ~PSRam_Allocator() throw() { }

    pointer
      allocate(size_type __n, const void* = 0)
      { 
	if (__n > this->max_size())
	  std::__throw_bad_alloc();
      Serial.printf("psram malloc  \n");
      

	return static_cast<_Tp*>(ps_malloc(__n * sizeof(_Tp)));
    }
};

template<typename _Tp>
    class new_allocator
    {
    public:
      typedef size_t     size_type;
      typedef ptrdiff_t  difference_type;
      typedef _Tp*       pointer;
      typedef const _Tp* const_pointer;
      typedef _Tp&       reference;
      typedef const _Tp& const_reference;
      typedef _Tp        value_type;

      template<typename _Tp1>
        struct rebind
        { typedef new_allocator<_Tp1> other; };

#if __cplusplus >= 201103L
      // _GLIBCXX_RESOLVE_LIB_DEFECTS
      // 2103. propagate_on_container_move_assignment
      typedef std::true_type propagate_on_container_move_assignment;
#endif

      new_allocator() _GLIBCXX_USE_NOEXCEPT { }

      new_allocator(const new_allocator&) _GLIBCXX_USE_NOEXCEPT { }

      template<typename _Tp1>
        new_allocator(const new_allocator<_Tp1>&) _GLIBCXX_USE_NOEXCEPT { }

      ~new_allocator() _GLIBCXX_USE_NOEXCEPT { }

      pointer
      address(reference __x) const _GLIBCXX_NOEXCEPT
      { return std::__addressof(__x); }

      const_pointer
      address(const_reference __x) const _GLIBCXX_NOEXCEPT
      { return std::__addressof(__x); }

      // NB: __n is permitted to be 0.  The C++ standard says nothing
      // about what the return value is when __n == 0.
      pointer
      allocate(size_type __n, const void* = 0)
      { 
	if (__n > this->max_size())
	  std::__throw_bad_alloc();
    //   Serial.printf("ps_malloc \n");

	return static_cast<_Tp*>(ps_malloc(__n * sizeof(_Tp)));
      }

      // __p is not permitted to be a null pointer.
      void
      deallocate(pointer __p, size_type)
      { free(__p);
        // ::operator delete(__p);
         }

      size_type
      max_size() const _GLIBCXX_USE_NOEXCEPT
      { return size_t(-1) / sizeof(_Tp); }

#if __cplusplus >= 201103L
      template<typename _Up, typename... _Args>
        void
        construct(_Up* __p, _Args&&... __args)
	{   
        ::new((void *)__p) _Up(std::forward<_Args>(__args)...); }

      template<typename _Up>
        void 
        destroy(_Up* __p) { __p->~_Up(); }
#else
      // _GLIBCXX_RESOLVE_LIB_DEFECTS
      // 402. wrong new expression in [some_] allocator::construct
      void 
      construct(pointer __p, const _Tp& __val) 
      { ::new((void *)__p) _Tp(__val); }

      void 
      destroy(pointer __p) { __p->~_Tp(); }
#endif
    };

//   template<typename _Tp>
//     inline bool
//     operator==(const new_allocator<_Tp>&, const new_allocator<_Tp>&)
//     { return true; }
  
//   template<typename _Tp>
//     inline bool
//     operator!=(const new_allocator<_Tp>&, const new_allocator<_Tp>&)
//     { return false; }




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

#define SEARCH_RADIUS 3

bool ifPurple(int R, int G, int B){
    // if (R > 0xA5 && R > G && R - G < 40 && R - B < 5 && R - B > -5)
    //     return true;
    if (R > 0x95 && G > 0x95 && B > 0x95)
        return true;
    return false;
}

static void detect_around(Map &map, Bitmap &detected_mask, Dot &dot){
    // Serial.printf("init\n");
    Bitmap need_detect_mask(map.W, map.H, map.L);
    // Serial.printf("init 2\n");
    //Bitmap alredy_detect_mask(map.W, map.H, map.L);
    // Serial.printf("init 2\n");

    uint32_t lastX, lastY;

    Vector2u first_detect;
    
    first_detect.x = dot.x;
    first_detect.y = dot.y;
    lastX = dot.x;
    lastY = dot.y;
    
    std::list<Vector2u, new_allocator<Vector2u>> need_detect_vector(1, first_detect, new_allocator<Vector2u>());

    // Serial.printf("init 2\n");

    while (need_detect_vector.size() > 0){
        // Serial.printf("init 3\n");
        auto elem = need_detect_vector.begin();
        const uint32_t x = elem->x;
        const uint32_t y = elem->y;
        const int R = map.getMap(x, y)->R;
        const int G = map.getMap(x, y)->G;
        const int B = map.getMap(x, y)->B;
         
        
        if (ifPurple(R, G, B) && !detected_mask.getCell(x, y)){
            //log_d("x = %u, y = %u, size = %u", x, y, need_detect_vector.size());
           // alredy_detect_mask.setCell(x, y, true);
			detected_mask.setCell(x,y, true);
			if (dot.x > x)
            	dot.x = x;
        	if (dot.y > y)
            	dot.y = y;
        	if (lastX < x)
            	lastX = x;
        	if (lastY < y)
            	lastY = y;
			const int beginY = ((int)y - SEARCH_RADIUS < 1 ? 1 : y - SEARCH_RADIUS);
			const int endY = (y + SEARCH_RADIUS > map.H ? map.H : y + SEARCH_RADIUS);
			const int beginX = ((int)x - SEARCH_RADIUS < 1 ? 1 : x - SEARCH_RADIUS);
			const int endX = (x + SEARCH_RADIUS > map.W ? map.W : x + SEARCH_RADIUS);
            for(uint32_t yy = beginY; yy <= endY; ++yy){
                for(uint32_t xx = beginX; xx <= endX; ++xx){
                    if (!detected_mask.getCell(xx, yy)/* && !alredy_detect_mask.getCell(xx, yy)*/ && !need_detect_mask.getCell(xx, yy)){
                        Vector2u detect_elem = {xx, yy};
                        need_detect_vector.push_back(detect_elem);
                        need_detect_mask.setCell(xx, yy, true);
                    }
                }    
            }

            // if (x > 1 && !detected_mask.getCell(x - 1, y) && !alredy_detect_mask.getCell(x - 1, y)){
            //     Vector2u detect_elem = {x - 1, y};
            //     need_detect_vector.push_back(detect_elem);
            //     need_detect_mask.setCell(x - 1, y, true);
            // }
            // if (x > 1 && y > 1 && !detected_mask.getCell(x - 1, y - 1) && !alredy_detect_mask.getCell(x - 1, y - 1)){
            //     Vector2u detect_elem = {x - 1, y - 1};
            //     need_detect_vector.push_back(detect_elem);
            //     need_detect_mask.setCell(x - 1, y - 1, true);
            // }
            // if (y > 1 && !detected_mask.getCell(x, y - 1) && !alredy_detect_mask.getCell(x, y - 1)){
            //     Vector2u detect_elem = {x, y - 1};
            //     need_detect_vector.push_back(detect_elem);
            //     need_detect_mask.setCell(x, y - 1, true);
            // }
            // if (x < map.W && y > 1 && !detected_mask.getCell(x + 1, y - 1) && !alredy_detect_mask.getCell(x + 1, y - 1)){
            //     Vector2u detect_elem = {x + 1, y - 1};
            //     need_detect_vector.push_back(detect_elem);
            //     need_detect_mask.setCell(x + 1, y - 1, true);
            // }
            // if (x < map.W && !detected_mask.getCell(x + 1, y) && !alredy_detect_mask.getCell(x + 1, y)){
            //     Vector2u detect_elem = {x + 1, y};
            //     need_detect_vector.push_back(detect_elem);
            //     need_detect_mask.setCell(x + 1, y, true);
            // }
            // if (x < map.W && y < map.H && !detected_mask.getCell(x + 1, y + 1) && !alredy_detect_mask.getCell(x + 1, y + 1)){
            //     Vector2u detect_elem = {x + 1, y + 1};
            //     need_detect_vector.push_back(detect_elem);
            //     need_detect_mask.setCell(x + 1, y + 1, true);
            // }
            // if (y < map.H && !detected_mask.getCell(x, y + 1) && !alredy_detect_mask.getCell(x, y + 1)){
            //     Vector2u detect_elem = {x, y + 1};
            //     need_detect_vector.push_back(detect_elem);
            //     need_detect_mask.setCell(x, y + 1, true);
            // }
            // if (x > 1 && y < map.H && !detected_mask.getCell(x - 1, y + 1) && !alredy_detect_mask.getCell(x - 1, y + 1)){
            //     Vector2u detect_elem = {x - 1, y + 1};
            //     need_detect_vector.push_back(detect_elem);
            //     need_detect_mask.setCell(x - 1, y + 1, true);
            // }
        }
        need_detect_vector.erase(elem);
    }

    // uint32_t xx = 1;
    // uint32_t yy = 1;

    // for (uint32_t y = 1; y <= map.H; ++y){
    //     for (uint32_t x = 1; x <= map.W; ++x){
    //         if (alredy_detect_mask.getCell(x, y)){
    //             if (dot.x > x)
    //                 dot.x = x;
    //             if (dot.y > y)
    //                 dot.y = y;
    //             if (xx < x)
    //                 xx = x;
    //             if (yy < y)
    //                 yy = y; 
    //         }
    //     }
    // }
    dot.w = lastX - dot.x;
    dot.h = lastY - dot.y;
    
}

int limiter = 0;
	

static void dotsDetector(Map map, std::vector<Dot, new_allocator<Dot>> &dots){

    Bitmap bitmap(map.W, map.H, map.L);
    // bool *bitmap = new bool[map.L];
    // Serial.printf("in 1\n");
    for (auto dot : dots){
        for ( int y = dot.y; y <= dot.y + dot.h; ++y){
            for (int x = dot.x; x <= dot.x + dot.w; ++x){
                bitmap.setCell(x, y, true);
            }
        }
    }
    
    // for (int i = 0; i < map.H; ++i){
    //     for (int j = 0; j < map.W; ++j){
    //         Serial.printf("(R%i.3|G%i.3|B%i.3)", map.map[(i * map.H) + j].R, map.map[(i * map.H) + j].G, map.map[(i * map.H) + j].B);
    //     }
    //     Serial.print("\n");
    // }
    int h = map.H / 15;

    
    // Serial.printf("in 2\n");
    for (int y = h * limiter + 1; y <= h * (limiter + 1); ++y){
        //Serial.printf("in 3\n");
        for (int x = 1; x <= map.W; ++x){
           // Serial.printf("in 4\n");
           const int R = map.getMap(x, y)->R;
           const int G = map.getMap(x, y)->G;
           const int B = map.getMap(x, y)->B;
            if (ifPurple(R, G, B) &&   bitmap.getCell(x, y) == false){
                if (dots.size() > 10)
                    return;
                //Serial.printf("in 5\n");
                Dot dot;
                dot.x = x;
                dot.y = y;
                dot.w = 0;
                dot.h = 0;
                detect_around(map, bitmap, dot);
                // bool Break = false;
                // for (int ii = y;ii < map.H; ii++){
                //     for (int jj = x; jj >= 0; --jj){
                //         if (map.map[(ii * map.H) + jj].R > 0x85 && bitmap[(ii * map.H) + jj] == false){
                            
                //             dot.x = jj;
                //         }
                //         else
                //             break;
                //     }
                //     Break = true;
                //     for (int jj = j; jj < map.W; jj++){
                        
                //         if (map.map[(ii * map.H) + jj].R > 0x85 && bitmap[(ii * map.H) + jj] == false ){
                //             if (dot.w < jj - dot.x)
                //                 dot.w = jj - dot.x;
                //             Break = false;
                //         }
                //         else {
                //             if (dot.x + dot.w < jj){
                //                 break;
                //             }
                //         }
                //     }
                //     if (Break){
                //         dot.h = ii - dot.y;
                //         break;
                //     }
                    
                    
                // }
                // for (int ii = dot.y; ii < dot.y + dot.h && ii <= map.H; ii++){
                //     for (int jj = dot.x; jj < dot.x + dot.w && jj <= map.W; jj++){
                //         bitmap.setCell(jj, ii, true);
                //     }
                // }
                // Serial.printf("in 6\n");
                // if (dot.h > 2 || dot.w > 2)
                    dots.push_back(dot);
                

            }
            
        }
    }
    ++limiter;
    if (limiter == 15)
        limiter = 0;

}

// static void draw_face_boxes(dl_matrix3du_t *image_matrix, box_array_t *boxes, int face_id){
//     int x, y, w, h, i;
//     uint32_t color = FACE_COLOR_YELLOW;
//     if(face_id < 0){
//         color = FACE_COLOR_RED;
//     } else if(face_id > 0){
//         color = FACE_COLOR_GREEN;
//     }
//     fb_data_t fb;
//     fb.width = image_matrix->w;
//     fb.height = image_matrix->h;
//     fb.data = image_matrix->item;
//     fb.bytes_per_pixel = 3;
//     fb.format = FB_BGR888;
//     for (i = 0; i < boxes->len; i++){
//         // rectangle box
//         x = (int)boxes->box[i].box_p[0];
//         y = (int)boxes->box[i].box_p[1];
//         w = (int)boxes->box[i].box_p[2] - x + 1;
//         h = (int)boxes->box[i].box_p[3] - y + 1;
//         fb_gfx_drawFastHLine(&fb, x, y, w, color);
//         fb_gfx_drawFastHLine(&fb, x, y+h-1, w, color);
//         fb_gfx_drawFastVLine(&fb, x, y, h, color);
//         fb_gfx_drawFastVLine(&fb, x+w-1, y, h, color);
// #if 0
//         // landmark
//         int x0, y0, j;
//         for (j = 0; j < 10; j+=2) {
//             x0 = (int)boxes->landmark[i].landmark_p[j];
//             y0 = (int)boxes->landmark[i].landmark_p[j+1];
//             fb_gfx_fillRect(&fb, x0, y0, 3, 3, color);
//         }
// #endif
//     }
// }

void DrawLine(Map &map, float x1, float y1, float x2, float y2){
    float xdiff = (x2 - x1);
	float ydiff = (y2 - y1);
    RGB rgb = {0, 255, 0};

	if(xdiff == 0.0f && ydiff == 0.0f) {
        map.setMap(x1, y1, rgb);
		// SetPixel(x1, y1, color1);
		return;
	}

	if(fabs(xdiff) > fabs(ydiff)) {
		float xmin, xmax;

		// set xmin to the lower x value given
		// and xmax to the higher value
		if(x1 < x2) {
			xmin = x1;
			xmax = x2;
		} else {
			xmin = x2;
			xmax = x1;
		}

		// draw line in terms of y slope
		float slope = ydiff / xdiff;
		for(float x = xmin; x <= xmax; x += 1.0f) {
			float y = y1 + ((x - x1) * slope);
			// Color color = color1 + ((color2 - color1) * ((x - x1) / xdiff));
			// SetPixel(x, y, color);
            map.setMap(x, y, rgb);
		}
	} else {
		float ymin, ymax;

		// set ymin to the lower y value given
		// and ymax to the higher value
		if(y1 < y2) {
			ymin = y1;
			ymax = y2;
		} else {
			ymin = y2;
			ymax = y1;
		}

		// draw line in terms of x slope
		float slope = xdiff / ydiff;
		for(float y = ymin; y <= ymax; y += 1.0f) {
			float x = x1 + ((y - y1) * slope);
			// Color color = color1 + ((color2 - color1) * ((y - y1) / ydiff));
			// SetPixel(x, y, color);
            map.setMap(x, y, rgb);
		}
	}
}

std::vector<Dot, new_allocator<Dot>> dots;

static void draw_crosses(dl_matrix3du_t *image_matrix, std::vector<Dot, new_allocator<Dot>> &dots)
{
    int x, y, w, h;
    // Serial.printf("cross in 1");
    uint32_t color = FACE_COLOR_GREEN;
    fb_data_t fb;
    fb.width = image_matrix->w;
    fb.height = image_matrix->h;
    fb.data = image_matrix->item;
    fb.bytes_per_pixel = 3;
    fb.format = FB_BGR888;
    // fb_gfx_drawFastHLine(&fb, 0, 0, fb.width / 2, color);
    // Serial.printf("cross in 2\n"); 
    for (auto dot : dots)
    {
        x = dot.x;
        y = dot.y;
        w = dot.w;
        h = dot.h;
        if (x - 1 + w / 2 - 5 < 0)
            x = 1 + 5;
        if (y - 1 + h / 2 - 5 < 0)
            y = 1 + 5;
        if (x - 1 + w / 2 + 5 >= fb.width)
            x = fb.width - w / 2 - 1 - 5;
        if (y - 1 + h / 2 + 5 >= fb.height)
            y = fb.height - h / 2 - 1 - 5;
        // Serial.printf("x = %i\n y = %i\n w = %i\n h = %i\n", dot.x, dot.y, dot.w, dot.h);
        // if (dot.x < 10 || dot.y < 10 || dot.y > fb.height - 10 || dot.x > fb.width - 10)
        //     continue;

        fb_gfx_drawFastHLine(&fb, x - 1 + w / 2 - 5, y - 1 + h / 2, 10, color);
        // log_d("H line %i", ((((dot.y + dot.h / 2) - 5) > 1) ? ((dot.y + dot.h / 2) - 5) : 1));
        fb_gfx_drawFastVLine(&fb, x - 1 + w / 2, y - 1 + h / 2 - 5, 10, color);
        // log_d("V line");
        // Serial.printf("cross in 4");
    }
    // Serial.printf("cross in 5");
}

#define PIXEL_SHIFT 3

void dotsTrack(Map &map){
   
    Bitmap already_detected(map.W, map.H, map.L);

    for (auto dot = dots.begin(); dot != dots.end(); ){ 
        // Serial.printf("dot x = %u, y = %u, w = %u, h = %u", dot->x,  dot->y,  dot->w, dot->h);     
        bool detected = false;
        int x = (int)dot->x - PIXEL_SHIFT;
        int y = (int)dot->y - PIXEL_SHIFT;
        int h = (int)dot->y + (int)dot->h + PIXEL_SHIFT;
        int w = (int)dot->x + (int)dot->w + PIXEL_SHIFT;
    //    Serial.printf("mod dot x = %u, y = %u, w = %u, h = %u\n", x,  y,  w, h);
        if (x < 1)
            x = 1;
        if (y < 1)
            y = 1;
        if (h > map.H)
            h = map.H;
        if (w > map.W)
            w = map.W;
        // Serial.printf("mod2 dot x = %i, y = %i, w = %i, h = %i\n", x,  y,  w, h);
        dot->x = w;
        dot->y = h;
        int lastX = x;
        int lastY = y;
        for (int yy = y; yy <= h; ++yy){
            // Serial.printf("y 1\n");
            for (int xx = x; xx <= w; ++xx){
            //    Serial.printf("x 1\n");
                const int R = map.getMap(xx, yy)->R;
                const int G = map.getMap(xx, yy)->G;
                const int B = map.getMap(xx, yy)->B;
                // RGB *rgb = map.getMap(x, y);
                if (ifPurple(R, G, B) && !already_detected.getCell(xx, yy)){
                //    Serial.printf("in x = %i, y = %i", x, y);
                    if (dot->x > xx)
            	        dot->x = xx;
        	        if (dot->y > y)
            	        dot->y = yy;
        	        if (lastX < xx)
            	        lastX = xx;
        	        if (lastY < yy)
            	        lastY = yy;
                    detected = true;
                }
            }
        }
        
        // Serial.printf("last x = %u, y = %u\n", lastX, lastY);
        dot->w = lastX - dot->x;
        dot->h = lastY - dot->y;
        if (detected){
        //    Serial.printf("dot x = %u, y = %u, w = %u, h = %u\n", dot->x,  dot->y,  dot->w, dot->h) ;
            DrawLine(map, dot->x, dot->y, dot->x + dot->w, dot->y + dot->h);
            DrawLine(map, dot->x, dot->y + dot->h, dot->x + dot->w, dot->y);
            for (int yy = dot->y; yy <= dot->y + dot->h; ++yy){
                for (int xx = dot->x; xx <= dot->x + dot->w; ++xx){
                    already_detected.setCell(xx, yy, true);
                }
            }
            ++dot;
        }
        else{
            dot = dots.erase(dot);
        }
    }
}

void irdetector(camera_fb_t * fb){
    const size_t lenth = fb->len;
    const size_t width = fb->width;
    const size_t height = fb->height;
    
    dl_matrix3du_t *image_matrix = NULL;

    
    Map map(width, height, lenth);
    map.map = fb->buf;
    dotsTrack(map);
    // Serial.printf("with = %u, height = %u, len = %u\n", fb->width, fb->height, fb->len);
    // Serial.printf("dots 1");
    dotsDetector(map, dots);
    // Serial.printf("dots 2");
    // image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
    // fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);
    // Serial.printf("dots 3");
	// Serial.printf("heap size = %u, psram size = %u", ESP.getFreeHeap(), ESP.getFreePsram());
    int x, y, w, h;
    for (auto dot : dots)
    {
        x = dot.x;
        y = dot.y;
        w = dot.w;
        h = dot.h;
        if (x - 1 + w / 2 - 5 < 0)
            x = 1 + 5;
        if (y - 1 + h / 2 - 5 < 0)
            y = 1 + 5;
        if (x - 1 + w / 2 + 5 >= fb->width)
            x = fb->width - w / 2 - 1 - 5;
        if (y - 1 + h / 2 + 5 >= fb->height)
            y = fb->height - h / 2 - 1 - 5;
        // Serial.printf("x = %i\n y = %i\n w = %i\n h = %i\n", dot.x, dot.y, dot.w, dot.h);
        // if (dot.x < 10 || dot.y < 10 || dot.y > fb.height - 10 || dot.x > fb.width - 10)
        //     continue;
        DrawLine(map, x - 1 + w / 2 - 5, y - 1 + h / 2, x - 1 + w / 2 + 5, y - 1 + h / 2);
        DrawLine(map, x - 1 + w / 2, y - 1 + h / 2 - 5, x - 1 + w / 2, y - 1 + h / 2 + 5);
        // fb_gfx_drawFastHLine(&fb, x - 1 + w / 2 - 5, y - 1 + h / 2, 10, color);
        // // log_d("H line %i", ((((dot.y + dot.h / 2) - 5) > 1) ? ((dot.y + dot.h / 2) - 5) : 1));
        // fb_gfx_drawFastVLine(&fb, x - 1 + w / 2, y - 1 + h / 2 - 5, 10, color);
        // log_d("V line");
        // Serial.printf("cross in 4");
    }
    
    // draw_crosses(image_matrix, dots);
    // fmt2rgb888(image_matrix->item, fb->len, fb->format, fb->buf);
    // dl_matrix3du_free(image_matrix);
    // Serial.printf("dots 4");
    // Serial.printf("format %i\n", *((int*)(0x3f80e364)));

//     for (int i = 0; i < lenth; i += 3)
//     {
//         const uint8_t RED = i+2;
//         const uint8_t GREEN = i + 1;
//         const uint8_t BLUE = i;
//         // Serial.printf("r%ug%ub%u\n",fb->buf[RED], fb->buf[GREEN], fb->buf[BLUE]);
//         if (fb->buf[RED] > 69)
//         {
//             fb->buf[RED] = 0;
//             fb->buf[GREEN] = 255;
//             fb->buf[BLUE] = 0;
//         }
//     }
}

// #include "esp_heap_caps.h"

static esp_err_t stream_handler(httpd_req_t *req){
	// heap_caps_check_integrity_all(1);
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t * _jpg_buf = NULL;
    char * part_buf[64];
    dl_matrix3du_t *image_matrix = NULL;
    bool detected = false;
    int face_id = 0;
    int64_t fr_start = 0;
    int64_t fr_ready = 0;
    int64_t fr_face = 0;
    int64_t fr_recognize = 0;
    int64_t fr_encode = 0;

    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while(true){
        detected = false;
        face_id = 0;
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            res = ESP_FAIL;
        } else {
            irdetector(fb);
            fr_start = esp_timer_get_time();
            fr_ready = fr_start;
            fr_face = fr_start;
            fr_encode = fr_start;
            fr_recognize = fr_start;
            log_d("detect0");
            if(!detection_enabled || fb->width > 400){
                log_d("detect1");
                if(fb->format != PIXFORMAT_JPEG){
                    bool jpeg_converted = frame2jpg(fb, 20, &_jpg_buf, &_jpg_buf_len);
                   // bool jpeg_converted = frame2bmp(fb, &_jpg_buf, &_jpg_buf_len);
                    esp_camera_fb_return(fb);
                    fb = NULL;
                    log_d("detect2");
                    if(!jpeg_converted){
                        Serial.println("JPEG compression failed");
                        res = ESP_FAIL;
                    }
                } else {
                    _jpg_buf_len = fb->len;
                    _jpg_buf = fb->buf;
                }
            } else {

                image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);

                if (!image_matrix) {
                    Serial.println("dl_matrix3du_alloc failed");
                    res = ESP_FAIL;
                } else {
                    if(!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)){
                        Serial.println("fmt2rgb888 failed");
                        res = ESP_FAIL;
                    } else {
                        fr_ready = esp_timer_get_time();
                        box_array_t *net_boxes = NULL;
                        if(detection_enabled){
                            net_boxes = face_detect(image_matrix, &mtmn_config);
                        }
                        fr_face = esp_timer_get_time();
                        fr_recognize = fr_face;
                        if (net_boxes || fb->format != PIXFORMAT_JPEG){
                            if(net_boxes){
                                detected = true;
                                if(recognition_enabled){
                                    face_id = run_face_recognition(image_matrix, net_boxes);
                                }
                                fr_recognize = esp_timer_get_time();
                                draw_face_boxes(image_matrix, net_boxes, face_id);
                                free(net_boxes->score);
                                free(net_boxes->box);
                                free(net_boxes->landmark);
                                free(net_boxes);
                            }
                            if(!fmt2jpg(image_matrix->item, fb->width*fb->height*3, fb->width, fb->height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len)){
                                Serial.println("fmt2jpg failed");
                                res = ESP_FAIL;
                            }
                            esp_camera_fb_return(fb);
                            fb = NULL;
                        } else {
                            _jpg_buf = fb->buf;
                            _jpg_buf_len = fb->len;
                        }
                        fr_encode = esp_timer_get_time();
                    }
                    dl_matrix3du_free(image_matrix);
                }
            }
        }
        log_d("detect3");
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        log_d("detect4");
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        log_d("detect5");
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        log_d("detect6");
        if(fb){
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        } else if(_jpg_buf){
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        log_d("detect7");
        if(res != ESP_OK){
            break;
        }
        log_d("detect8");
        int64_t fr_end = esp_timer_get_time();

        int64_t ready_time = (fr_ready - fr_start)/1000;
        int64_t face_time = (fr_face - fr_ready)/1000;
        int64_t recognize_time = (fr_recognize - fr_face)/1000;
        int64_t encode_time = (fr_encode - fr_recognize)/1000;
        int64_t process_time = (fr_encode - fr_start)/1000;
        
        int64_t frame_time = fr_end - last_frame;
        log_d("detect9");
        last_frame = fr_end;
        frame_time /= 1000;
        uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
        Serial.printf("MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps), %u+%u+%u+%u=%u %s%d\n",
            (uint32_t)(_jpg_buf_len),
            (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time,
            avg_frame_time, 1000.0 / avg_frame_time,
            (uint32_t)ready_time, (uint32_t)face_time, (uint32_t)recognize_time, (uint32_t)encode_time, (uint32_t)process_time,
            (detected)?"DETECTED ":"", face_id
        );
    }

    last_frame = 0;
    return res;
}

static esp_err_t cmd_handler(httpd_req_t *req){
    char*  buf;
    size_t buf_len;
    char variable[32] = {0,};
    char value[32] = {0,};

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        if(!buf){
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
                httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
            } else {
                free(buf);
                httpd_resp_send_404(req);
                return ESP_FAIL;
            }
        } else {
            free(buf);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        free(buf);
    } else {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    int val = atoi(value);
    sensor_t * s = esp_camera_sensor_get();
    int res = 0;

    if(!strcmp(variable, "framesize")) {
        if(s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)val);
    }
    else if(!strcmp(variable, "quality")) res = s->set_quality(s, val);
    else if(!strcmp(variable, "contrast")) res = s->set_contrast(s, val);
    else if(!strcmp(variable, "brightness")) res = s->set_brightness(s, val);
    else if(!strcmp(variable, "saturation")) res = s->set_saturation(s, val);
    else if(!strcmp(variable, "gainceiling")) res = s->set_gainceiling(s, (gainceiling_t)val);
    else if(!strcmp(variable, "colorbar")) res = s->set_colorbar(s, val);
    else if(!strcmp(variable, "awb")) res = s->set_whitebal(s, val);
    else if(!strcmp(variable, "agc")) res = s->set_gain_ctrl(s, val);
    else if(!strcmp(variable, "aec")) res = s->set_exposure_ctrl(s, val);
    else if(!strcmp(variable, "hmirror")) res = s->set_hmirror(s, val);
    else if(!strcmp(variable, "vflip")) res = s->set_vflip(s, val);
    else if(!strcmp(variable, "awb_gain")) res = s->set_awb_gain(s, val);
    else if(!strcmp(variable, "agc_gain")) res = s->set_agc_gain(s, val);
    else if(!strcmp(variable, "aec_value")) res = s->set_aec_value(s, val);
    else if(!strcmp(variable, "aec2")) res = s->set_aec2(s, val);
    else if(!strcmp(variable, "dcw")) res = s->set_dcw(s, val);
    else if(!strcmp(variable, "bpc")) res = s->set_bpc(s, val);
    else if(!strcmp(variable, "wpc")) res = s->set_wpc(s, val);
    else if(!strcmp(variable, "raw_gma")) res = s->set_raw_gma(s, val);
    else if(!strcmp(variable, "lenc")) res = s->set_lenc(s, val);
    else if(!strcmp(variable, "special_effect")) res = s->set_special_effect(s, val);
    else if(!strcmp(variable, "wb_mode")) res = s->set_wb_mode(s, val);
    else if(!strcmp(variable, "ae_level")) res = s->set_ae_level(s, val);
    else if(!strcmp(variable, "face_detect")) {
        detection_enabled = val;
        if(!detection_enabled) {
            recognition_enabled = 0;
        }
    }
    else if(!strcmp(variable, "face_enroll")) is_enrolling = val;
    else if(!strcmp(variable, "face_recognize")) {
        recognition_enabled = val;
        if(recognition_enabled){
            detection_enabled = val;
        }
    }
    else {
        res = -1;
    }

    if(res){
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t status_handler(httpd_req_t *req){
    static char json_response[1024];

    sensor_t * s = esp_camera_sensor_get();
    char * p = json_response;
    *p++ = '{';

    p+=sprintf(p, "\"framesize\":%u,", s->status.framesize);
    p+=sprintf(p, "\"quality\":%u,", s->status.quality);
    p+=sprintf(p, "\"brightness\":%d,", s->status.brightness);
    p+=sprintf(p, "\"contrast\":%d,", s->status.contrast);
    p+=sprintf(p, "\"saturation\":%d,", s->status.saturation);
    p+=sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
    p+=sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
    p+=sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
    p+=sprintf(p, "\"awb\":%u,", s->status.awb);
    p+=sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
    p+=sprintf(p, "\"aec\":%u,", s->status.aec);
    p+=sprintf(p, "\"aec2\":%u,", s->status.aec2);
    p+=sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
    p+=sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
    p+=sprintf(p, "\"agc\":%u,", s->status.agc);
    p+=sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
    p+=sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
    p+=sprintf(p, "\"bpc\":%u,", s->status.bpc);
    p+=sprintf(p, "\"wpc\":%u,", s->status.wpc);
    p+=sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
    p+=sprintf(p, "\"lenc\":%u,", s->status.lenc);
    p+=sprintf(p, "\"vflip\":%u,", s->status.vflip);
    p+=sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
    p+=sprintf(p, "\"dcw\":%u,", s->status.dcw);
    p+=sprintf(p, "\"colorbar\":%u,", s->status.colorbar);
    p+=sprintf(p, "\"face_detect\":%u,", detection_enabled);
    p+=sprintf(p, "\"face_enroll\":%u,", is_enrolling);
    p+=sprintf(p, "\"face_recognize\":%u", recognition_enabled);
    *p++ = '}';
    *p++ = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t index_handler(httpd_req_t *req){
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        return httpd_resp_send(req, (const char *)index_ov3660_html_gz, index_ov3660_html_gz_len);
    }
    return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}

void startCameraServer(){
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t status_uri = {
        .uri       = "/status",
        .method    = HTTP_GET,
        .handler   = status_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t cmd_uri = {
        .uri       = "/control",
        .method    = HTTP_GET,
        .handler   = cmd_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t capture_uri = {
        .uri       = "/capture",
        .method    = HTTP_GET,
        .handler   = capture_handler,
        .user_ctx  = NULL
    };

   httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };


    ra_filter_init(&ra_filter, 20);
    
    mtmn_config.type = FAST;
    mtmn_config.min_face = 80;
    mtmn_config.pyramid = 0.707;
    mtmn_config.pyramid_times = 4;
    mtmn_config.p_threshold.score = 0.6;
    mtmn_config.p_threshold.nms = 0.7;
    mtmn_config.p_threshold.candidate_number = 20;
    mtmn_config.r_threshold.score = 0.7;
    mtmn_config.r_threshold.nms = 0.7;
    mtmn_config.r_threshold.candidate_number = 10;
    mtmn_config.o_threshold.score = 0.7;
    mtmn_config.o_threshold.nms = 0.7;
    mtmn_config.o_threshold.candidate_number = 1;
    
    face_id_init(&id_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
    
    Serial.printf("Starting web server on port: '%d'\n", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
        httpd_register_uri_handler(camera_httpd, &status_uri);
        httpd_register_uri_handler(camera_httpd, &capture_uri);
    }

    config.server_port += 1;
    config.ctrl_port += 1;
    Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}