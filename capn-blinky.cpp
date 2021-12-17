
#include <stdint.h>
#include <memory.h>
#include <algorithm>
#include <tuple>

#ifdef USE_HAL_DRIVER
#include "stm32l0xx_hal.h"
extern "C" SPI_HandleTypeDef hspi1;
extern "C" DMA_HandleTypeDef hdma_spi1_tx;
#else  // #ifdef USE_HAL_DRIVER
struct TIM_HandleTypeDef;
#include <stdio.h>
#include <thread>
#include <chrono>
#ifdef WIN32
#include <Windows.h>
#endif //#ifdef WIN32
#endif  // #ifdef USE_HAL_DRIVER

template<size_t fbits> class fixed32 {

public: 

    constexpr fixed32() {
        raw = 0;
    }

    template <typename T, typename std::enable_if<std::is_integral<T>::value>::type* = nullptr>
    constexpr inline explicit fixed32(T a) noexcept {
        raw = static_cast<int32_t>(a << fbits);
    }

    template <typename T, typename std::enable_if<std::is_floating_point<T>::value>::type* = nullptr>
    consteval inline explicit fixed32(T a) noexcept {
        raw = static_cast<int32_t>(a * float(1L << fbits));
    }

    template <typename T, typename std::enable_if<std::is_floating_point<T>::value>::type* = nullptr>
    constexpr inline explicit operator T() const noexcept
    {
        return static_cast<T>(raw) / T(1L << fbits);
    }

    template <typename T, typename std::enable_if<std::is_integral<T>::value>::type* = nullptr>
    constexpr inline explicit operator T() const noexcept
    {
        return static_cast<T>(raw >> fbits);
    }

    constexpr fixed32 &operator=(const fixed32& other) = default;

    constexpr fixed32 &operator+=(const fixed32 &b) {
        this->raw += b.raw;
        return *this;
    }

    constexpr fixed32 &operator-=(const fixed32 &b) {
        this->raw -= b.raw;
        return *this;
    }

    constexpr fixed32 &operator*=(const fixed32 &b) {
        auto value = (int64_t(this->raw) * int64_t(b.raw)) >> (fbits - 1);
        this->raw = value / 2 + (value & 1);
        return *this;
    }

    constexpr fixed32 &operator/=(const fixed32 &b) {
        auto value = (int64_t(this->raw) << (fbits + 1)) / b.raw;
        this->raw = value / 2 + (value & 1);
        return *this;
    }

    constexpr fixed32 &operator%=(const fixed32 &b) {
        this->raw = this->raw % b.raw;
        return *this;
    }

    constexpr fixed32 operator-() const {
        return -raw;
    }

    constexpr fixed32 operator+() const {
        return +raw;
    }

    constexpr fixed32 operator+(const fixed32 &b) const {
        fixed32 ret; 
        ret.raw = raw + b.raw;
        return ret;
    }

    constexpr fixed32 operator-(const fixed32 &b) const {
        fixed32 ret; 
        ret.raw = raw - b.raw;
        return ret;
    }

    constexpr fixed32 operator*(const fixed32 &b) const {
        auto value = (int64_t(this->raw) * int64_t(b.raw)) >> (fbits - 1);
        fixed32 ret; 
        ret.raw = value / 2 + (value & 1);
        return ret;
    }

    constexpr fixed32 operator/(const fixed32 &b) const {
        auto value = (int64_t(this->raw) << (fbits + 1)) / b.raw;
        fixed32 ret; 
        ret.raw = value / 2 + (value & 1);
        return ret;
    }

    constexpr fixed32 operator%(const fixed32 &b) const {
        fixed32 ret; 
        ret.raw = raw % b.raw;
        return ret;
    }

    constexpr fixed32 frac() const {
        fixed32 ret; 
        ret.raw = raw & ((1L << fbits) - 1);
        return ret;
    }

    constexpr fixed32 integral() const {
        fixed32 ret; 
        ret.raw = raw & ((~0) << fbits);
        return ret;
    }

    constexpr fixed32 abs() const {
        fixed32 ret; 
        int32_t mask = raw >> 31;
        ret.raw = ( mask ^ raw ) - mask;
        return ret;
    }

    constexpr fixed32 clamp(const fixed32 &lo, const fixed32 &hi) {
        return std::min(hi, std::max(lo, *this));
    }

    int32_t raw;
};

template <size_t fbits> constexpr inline bool operator==(const fixed32<fbits>& x, const fixed32<fbits>& y) noexcept {
    return x.raw == y.raw;
}

template <size_t fbits> constexpr inline bool operator!=(const fixed32<fbits>& x, const fixed32<fbits>& y) noexcept {
    return x.raw != y.raw;
}

template <size_t fbits> constexpr inline bool operator<(const fixed32<fbits>& x, const fixed32<fbits>& y) noexcept {
    return x.raw < y.raw;
}

template <size_t fbits> constexpr inline bool operator>(const fixed32<fbits>& x, const fixed32<fbits>& y) noexcept {
    return x.raw > y.raw;
}

template <size_t fbits> constexpr inline bool operator<=(const fixed32<fbits>& x, const fixed32<fbits>& y) noexcept {
    return x.raw <= y.raw;
}

template <size_t fbits> constexpr inline bool operator>=(const fixed32<fbits>& x, const fixed32<fbits>& y) noexcept {
    return x.raw >= y.raw;
}

static constexpr fixed32<24> lerp(fixed32<24> a, fixed32<24> b, fixed32<24> t) {
    return ( a + t * (b - a));
}

static constexpr fixed32<24> sin(fixed32<24> v) {
    constexpr fixed32<24> table[] = {
        fixed32<24>(+0.000000000000f),fixed32<24>(+0.098017140330f),fixed32<24>(+0.195090322016f),fixed32<24>(+0.290284677254f),
        fixed32<24>(+0.382683432365f),fixed32<24>(+0.471396736826f),fixed32<24>(+0.555570233020f),fixed32<24>(+0.634393284164f),
        fixed32<24>(+0.707106781187f),fixed32<24>(+0.773010453363f),fixed32<24>(+0.831469612303f),fixed32<24>(+0.881921264348f),
        fixed32<24>(+0.923879532511f),fixed32<24>(+0.956940335732f),fixed32<24>(+0.980785280403f),fixed32<24>(+0.995184726672f),
        fixed32<24>(+1.000000000000f),fixed32<24>(+0.995184726672f),fixed32<24>(+0.980785280403f),fixed32<24>(+0.956940335732f),
        fixed32<24>(+0.923879532511f),fixed32<24>(+0.881921264348f),fixed32<24>(+0.831469612302f),fixed32<24>(+0.773010453363f),
        fixed32<24>(+0.707106781186f),fixed32<24>(+0.634393284164f),fixed32<24>(+0.555570233019f),fixed32<24>(+0.471396736826f),
        fixed32<24>(+0.382683432365f),fixed32<24>(+0.290284677254f),fixed32<24>(+0.195090322016f),fixed32<24>(+0.098017140329f),
        fixed32<24>(-0.000000000000f),fixed32<24>(-0.098017140330f),fixed32<24>(-0.195090322016f),fixed32<24>(-0.290284677255f),
        fixed32<24>(-0.382683432365f),fixed32<24>(-0.471396736826f),fixed32<24>(-0.555570233020f),fixed32<24>(-0.634393284164f),
        fixed32<24>(-0.707106781187f),fixed32<24>(-0.773010453363f),fixed32<24>(-0.831469612303f),fixed32<24>(-0.881921264348f),
        fixed32<24>(-0.923879532511f),fixed32<24>(-0.956940335732f),fixed32<24>(-0.980785280403f),fixed32<24>(-0.995184726672f),
        fixed32<24>(-1.000000000000f),fixed32<24>(-0.995184726672f),fixed32<24>(-0.980785280403f),fixed32<24>(-0.956940335732f),
        fixed32<24>(-0.923879532511f),fixed32<24>(-0.881921264348f),fixed32<24>(-0.831469612302f),fixed32<24>(-0.773010453363f),
        fixed32<24>(-0.707106781186f),fixed32<24>(-0.634393284163f),fixed32<24>(-0.555570233019f),fixed32<24>(-0.471396736826f),
        fixed32<24>(-0.382683432365f),fixed32<24>(-0.290284677254f),fixed32<24>(-0.195090322016f),fixed32<24>(-0.098017140329f)
    };
    fixed32<24> i0 = table[(((v * fixed32<24>(10.1859163579f)).raw >> 24) + 0) % 64];
    fixed32<24> i1 = table[(((v * fixed32<24>(10.1859163579f)).raw >> 24) + 1) % 64];
    return lerp(i0, i1, v.frac());
}

static constexpr fixed32<24> cos(fixed32<24> v) {
    constexpr fixed32<24> table[] = {
        fixed32<24>(+1.000000000000f),fixed32<24>(+0.995184726672f),fixed32<24>(+0.980785280403f),fixed32<24>(+0.956940335732f),
        fixed32<24>(+0.923879532511f),fixed32<24>(+0.881921264348f),fixed32<24>(+0.831469612303f),fixed32<24>(+0.773010453363f),
        fixed32<24>(+0.707106781187f),fixed32<24>(+0.634393284164f),fixed32<24>(+0.555570233020f),fixed32<24>(+0.471396736826f),
        fixed32<24>(+0.382683432365f),fixed32<24>(+0.290284677254f),fixed32<24>(+0.195090322016f),fixed32<24>(+0.098017140329f),
        fixed32<24>(-0.000000000000f),fixed32<24>(-0.098017140330f),fixed32<24>(-0.195090322016f),fixed32<24>(-0.290284677255f),
        fixed32<24>(-0.382683432365f),fixed32<24>(-0.471396736826f),fixed32<24>(-0.555570233020f),fixed32<24>(-0.634393284164f),
        fixed32<24>(-0.707106781187f),fixed32<24>(-0.773010453363f),fixed32<24>(-0.831469612303f),fixed32<24>(-0.881921264348f),
        fixed32<24>(-0.923879532511f),fixed32<24>(-0.956940335732f),fixed32<24>(-0.980785280403f),fixed32<24>(-0.995184726672f),
        fixed32<24>(-1.000000000000f),fixed32<24>(-0.995184726672f),fixed32<24>(-0.980785280403f),fixed32<24>(-0.956940335732f),
        fixed32<24>(-0.923879532511f),fixed32<24>(-0.881921264348f),fixed32<24>(-0.831469612302f),fixed32<24>(-0.773010453363f),
        fixed32<24>(-0.707106781186f),fixed32<24>(-0.634393284163f),fixed32<24>(-0.555570233019f),fixed32<24>(-0.471396736826f),
        fixed32<24>(-0.382683432365f),fixed32<24>(-0.290284677254f),fixed32<24>(-0.195090322016f),fixed32<24>(-0.098017140329f),
        fixed32<24>(+0.000000000000f),fixed32<24>(+0.098017140330f),fixed32<24>(+0.195090322016f),fixed32<24>(+0.290284677255f),
        fixed32<24>(+0.382683432365f),fixed32<24>(+0.471396736826f),fixed32<24>(+0.555570233020f),fixed32<24>(+0.634393284164f),
        fixed32<24>(+0.707106781187f),fixed32<24>(+0.773010453363f),fixed32<24>(+0.831469612303f),fixed32<24>(+0.881921264349f),
        fixed32<24>(+0.923879532511f),fixed32<24>(+0.956940335732f),fixed32<24>(+0.980785280403f),fixed32<24>(+0.995184726672f),
    };
    fixed32<24> i0 = table[(((v * fixed32<24>(10.1859163579f)).raw >> 24) + 0) % 64];
    fixed32<24> i1 = table[(((v * fixed32<24>(10.1859163579f)).raw >> 24) + 1) % 64];
    return lerp(i0, i1, v.frac());
}

class hsv;

class rgb {
public:
    fixed32<24> r;
    fixed32<24> g;
    fixed32<24> b;

    constexpr rgb() :
        r(fixed32<24>(0.0f)),
        g(fixed32<24>(0.0f)),
        b(fixed32<24>(0.0f)){
    }

    rgb(const rgb &from) :
        r(from.r),
        g(from.g),
        b(from.b) {
    }

    explicit rgb(const uint32_t color) {
        r = static_cast<fixed32<24>>((color>>16)&0xFF) * fixed32<24>(1.0f/255.0f);
        g = static_cast<fixed32<24>>((color>> 8)&0xFF) * fixed32<24>(1.0f/255.0f);
        b = static_cast<fixed32<24>>((color>> 0)&0xFF) * fixed32<24>(1.0f/255.0f);
    }
    
    explicit rgb(const hsv &from);

    constexpr rgb &operator=(const rgb& other) = default;

    constexpr rgb(fixed32<24> _r, fixed32<24> _g, fixed32<24> _b) :
        r(_r),
        g(_g),
        b(_b) {
    }

    void set(fixed32<24> _r, fixed32<24> _g, fixed32<24> _b) {
        r = _r;
        g = _g;
        b = _b;
    }

    rgb &operator+=(const rgb &v) {
        r += v.r;
        g += v.g;
        b += v.b;
        return *this;
    }

    friend rgb operator+(rgb a, const rgb &_b) {
        a += _b;
        return a;
    }

    rgb &operator-=(const rgb &v) {
        r -= v.r;
        g -= v.g;
        b -= v.b;
        return *this;
    }

    friend rgb operator-(rgb a, const rgb &_b) {
        a -= _b;
        return a;
    }

    rgb &operator*=(const rgb &v) {
        r *= v.r;
        g *= v.g;
        b *= v.b;
        return *this;
    }

    friend rgb operator*(rgb a, const rgb &_b) {
        a *= _b;
        return a;
    }

    rgb &operator*=(fixed32<24> v) {
        r *= v;
        g *= v;
        b *= v;
        return *this;
    }

    friend rgb operator*(rgb a, fixed32<24> v) {
        a *= v;
        return a;
    }

    rgb &operator/=(const rgb &v) {
        r /= v.r;
        g /= v.g;
        b /= v.b;
        return *this;
    }

    friend rgb operator/(rgb a, const rgb &_b) {
        a /= _b;
        return a;
    }

    rgb &operator/=(fixed32<24> v) {
        r /= v;
        g /= v;
        b /= v;
        return *this;
    }

    friend rgb operator/(rgb a, fixed32<24> v) {
        a /= v;
        return a;
    }
};


class hsv {
public:
    fixed32<24> h;
    fixed32<24> s;
    fixed32<24> v;

    constexpr hsv() :
        h(fixed32<24>(0.0f)),
        s(fixed32<24>(0.0f)),
        v(fixed32<24>(0.0f)) {
    }

    constexpr hsv(fixed32<24> _h, fixed32<24> _s, fixed32<24> _v) :
        h(_h),
        s(_s),
        v(_v) {
    }

    constexpr hsv(const hsv &from) :
        h(from.h),
        s(from.s),
        v(from.v) {
    }

    constexpr hsv &operator=(const hsv& other) = default;

    constexpr explicit hsv(const rgb &from) {
        fixed32<24> hi = std::max(std::max(from.r, from.g), from.b);
        fixed32<24> lo = std::min(std::max(from.r, from.g), from.b);
        fixed32<24> d = hi - lo;

        h = fixed32<24>(0.0f);
        s = fixed32<24>(0.0f);
        v = hi;

        if ( ( v > fixed32<24>(0.00001f) ) &&
             ( d > fixed32<24>(0.00001f) ) ) {
            s = d / v;
            if( hi == from.r ) {
                h = fixed32<24>(60.0f/360.0f) * (from.g - from.b) / d + (from.g < from.b ? fixed32<24>(1.0f) : fixed32<24>(0.0f));
            }
            if( hi == from.g ) {
                h = fixed32<24>(60.0f/360.0f) * (from.b - from.r) / d + fixed32<24>(120.0f/360.0f);
            }
            if( hi == from.b ) {
                h = fixed32<24>(60.0f/360.0f) * (from.r - from.g) / d + fixed32<24>(240.0f/360.0f);
            }
        }
    }
};

rgb::rgb(const hsv &from) {
    fixed32<24> v = from.v;
    fixed32<24> h = from.h;
    fixed32<24> s = from.s;

    uint32_t rd = static_cast<uint32_t>( fixed32<24>(6.0f) * h.abs() );
    fixed32<24> f = h * fixed32<24>(6.0f) - fixed32<24>(rd);
    fixed32<24> p = v * (fixed32<24>(1.0f) - s);
    fixed32<24> q = v * (fixed32<24>(1.0f) - f * s);
    fixed32<24> t = v * (fixed32<24>(1.0f) - (fixed32<24>(1.0f) - f) * s);

    auto mod6 = []( uint32_t a ) {
        uint32_t c = a & 1;
        a = a >> 1;
        a = (a >> 16) + (a & 0xFFFF);
        a = (a >>  8) + (a & 0xFF);
        a = (a >>  4) + (a & 0xF);
        a = (a >>  2) + (a & 0x3);
        a = (a >>  2) + (a & 0x3);
        a = (a >>  2) + (a & 0x3);
        if (a > 2) a = a - 3;
        return c + (a << 1);
    };

    switch ( mod6(rd) ) {
        default:
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        case 5: r = v; g = p; b = q; break;
    }
}

class pseudo_random {
public:
    
    void set_seed(uint32_t seed) {
        uint32_t i;
        a = 0xf1ea5eed, b = c = d = seed;
        for (i=0; i<20; ++i) {
            (void)get();
        }
    }

    #define rot(x,k) (((x)<<(k))|((x)>>(32-(k))))
    uint32_t get() {
        uint32_t e = a - rot(b, 27);
        a = b ^ rot(c, 17);
        b = c + d;
        c = d + e;
        d = e + a;
        return d;
    }

    uint32_t get(uint32_t lower, uint32_t upper) {
        return (static_cast<int32_t>(get()) % (upper-lower)) + lower;
    }

private:
    uint32_t a; 
    uint32_t b; 
    uint32_t c; 
    uint32_t d; 

};

class Leds {
public:
    static constexpr size_t ledsN = 12;
    static constexpr size_t spiPaddingBytes = 64;

    static constexpr std::tuple<fixed32<24>, fixed32<24>, fixed32<24>, fixed32<24>> map[ledsN] = {
        {fixed32<24>(0.000000000000f), fixed32<24>(0.000000000000f), fixed32<24>(1.000000000000f), fixed32<24>(0.785398163398f)},
        {fixed32<24>(0.091880693216f), fixed32<24>(0.197963213135f), fixed32<24>(0.772332122866f), fixed32<24>(0.700511333882f)},
        {fixed32<24>(0.300700450525f), fixed32<24>(0.531201288579f), fixed32<24>(0.356442638758f), fixed32<24>(0.420808961092f)},
        {fixed32<24>(0.300700450525f), fixed32<24>(0.240855242648f), fixed32<24>(0.557331634801f), fixed32<24>(0.947448246866f)},
        {fixed32<24>(0.173073761058f), fixed32<24>(0.405824586927f), fixed32<24>(0.561155449758f), fixed32<24>(0.518578517019f)},
        {fixed32<24>(0.557302114506f), fixed32<24>(0.392627039385f), fixed32<24>(0.292085112127f), fixed32<24>(1.572261434407f)},
        {fixed32<24>(0.818573448650f), fixed32<24>(0.534500675465f), fixed32<24>(0.361259521513f), fixed32<24>(2.737267656516f)},
        {fixed32<24>(1.000000000000f), fixed32<24>(0.428920295126f), fixed32<24>(0.617041966382f), fixed32<24>(2.717833619879f)},
        {fixed32<24>(0.753592686376f), fixed32<24>(0.808349786969f), fixed32<24>(0.289876914397f), fixed32<24>(3.674825514208f)},
        {fixed32<24>(0.701141109540f), fixed32<24>(0.999350514393f), fixed32<24>(0.394305272030f), fixed32<24>(4.229642688686f)},
        {fixed32<24>(0.405965339209f), fixed32<24>(1.000000000000f), fixed32<24>(0.399002657007f), fixed32<24>(5.213568293370f)},
        {fixed32<24>(0.354993587425f), fixed32<24>(0.801751013198f), fixed32<24>(0.292326727774f), fixed32<24>(5.782303428300f)}
    };

    static Leds &instance();

    void transfer();

    static rgb led_buffer[ledsN * 3];

private:

    static uint8_t spi_buffer[ledsN * sizeof(uint16_t) * 3 * 4 + spiPaddingBytes];

    void init();
    bool initialized = false;
};

uint8_t Leds::spi_buffer[ledsN * sizeof(uint16_t) * 3 * 4 + spiPaddingBytes];
rgb Leds::led_buffer[ledsN * 3];

Leds &Leds::instance() {
    static Leds leds;
    if (!leds.initialized) {
        leds.initialized = true;
        leds.init();
    }
    return leds;
}

void Leds::init() {
}

void Leds::transfer() {
    uint32_t *ptr = reinterpret_cast<uint32_t *>(&spi_buffer[0]);
    for (size_t c = 0; c < spiPaddingBytes/(sizeof(uint32_t)*2); c++ ) {
        *ptr++ = 0;
    }

    for (size_t c = 0; c < ledsN; c++) {
        auto convert_to_one_wire_spi = [] (uint32_t *p, uint16_t v) {
            auto fix_ws2816 = [](uint16_t f) {
                if (f > 65535 - (256 - 32)) {
                    f = 65535 - (256 - 32);
                }
                if (f >= 32) {
                    f += 256 - 32;
                    if (f > 512) {
                        if (f > 512 + 32) {
                            f -= 32;
                        } else {
                            f = 512;
                        }
                    }
                }
                return f;
            };
            v = fix_ws2816(v);
            auto convert_half_to_spi = [] (uint8_t x) {
                return 0x88888888 | (((x >>  4) | (x <<  6) | (x << 16) | (x << 26)) & 0x04040404)|
                                    (((x >>  1) | (x <<  9) | (x << 19) | (x << 29)) & 0x40404040);
            };
            *p++ = convert_half_to_spi((v>>8)&0xFF);
            *p++ = convert_half_to_spi((v>>0)&0xFF);
            return p;
        };
        ptr = convert_to_one_wire_spi(ptr, ((led_buffer[c].g.clamp(fixed32<24>(0.0f), fixed32<24>(1.0f))) * fixed32<24>(1.0f/4096.0f)).raw);
        ptr = convert_to_one_wire_spi(ptr, ((led_buffer[c].r.clamp(fixed32<24>(0.0f), fixed32<24>(1.0f))) * fixed32<24>(1.0f/4096.0f)).raw);
        ptr = convert_to_one_wire_spi(ptr, ((led_buffer[c].b.clamp(fixed32<24>(0.0f), fixed32<24>(1.0f))) * fixed32<24>(1.0f/4096.0f)).raw);
    }

    for (size_t c = 0; c < spiPaddingBytes/(sizeof(uint32_t)*2); c++ ) {
        *ptr++ = 0;
    };

#ifdef USE_HAL_DRIVER
    HAL_SPI_DMAStop(&hspi1);
    HAL_SPI_Transmit_DMA(&hspi1, spi_buffer, sizeof(spi_buffer));
#endif  //#ifdef USE_HAL_DRIVER
}

class Model {
public:

    static Model &instance();

    size_t Pattern() const { return pattern; }
    void IncPattern() { pattern++; }

    void load();
    void save();
private:
    size_t pattern;

    void init();
    bool initialized = false;
};

Model &Model::instance() {
    static Model model;
    if (!model.initialized) {
        model.initialized = true;
        model.init();
    }
    return model;
}

void Model::load() {
#ifdef USE_HAL_DRIVER
    pattern = *((uint32_t *)DATA_EEPROM_BASE);
#endif  // #ifdef USE_HAL_DRIVER
}

void Model::save() {
#ifdef USE_HAL_DRIVER
    HAL_FLASHEx_DATAEEPROM_Unlock();
    HAL_FLASHEx_DATAEEPROM_Erase(DATA_EEPROM_BASE);
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, DATA_EEPROM_BASE, pattern);
    HAL_FLASHEx_DATAEEPROM_Lock();
#endif  // #ifdef USE_HAL_DRIVER
}

static pseudo_random rnd;

void Model::init() {
    load();
    rnd.set_seed(0xDEADBEEF);
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t) {
    Model::instance().IncPattern();
    Model::instance().save();
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *) {
    static fixed32<24> tick;
    switch(Model::instance().Pattern() % 7) {
        case    0: {
                    for (size_t c = 0; c < Leds::ledsN; c++) {
                        Leds::led_buffer[c] = rgb(hsv(fixed32<24>(0.6f), fixed32<24>(0.24f) + fixed32<24>(0.5f) * fixed32<24>(std::get<2>(Leds::map[c])), fixed32<24>(1.0f) - fixed32<24>(0.9f) * fixed32<24>(std::get<2>(Leds::map[c]))));
                    }
                } break;
        case    1: {
                    for (size_t c = 0; c < Leds::ledsN; c++) {
                        Leds::led_buffer[c] = rgb(hsv(fixed32<24>(0.9f), fixed32<24>(0.5f) + fixed32<24>(0.5f) * fixed32<24>(std::get<2>(Leds::map[c])), fixed32<24>(1.0f) - fixed32<24>(0.9f) * fixed32<24>(std::get<2>(Leds::map[c]))));
                    }
                } break;
        case    2: {
                    for (size_t c = 0; c < Leds::ledsN; c++) {
                        Leds::led_buffer[c] = rgb(hsv(fixed32<24>(0.1f), fixed32<24>(0.5f) + fixed32<24>(0.5f) * fixed32<24>(std::get<2>(Leds::map[c])), fixed32<24>(1.0f) - fixed32<24>(0.9f) * fixed32<24>(std::get<2>(Leds::map[c]))));
                    }
                } break;
        case    3: {
                    for (size_t c = 0; c < Leds::ledsN; c++) {
                        fixed32<24> h = fixed32<24>(1.0f) - (fixed32<24>(tick) * fixed32<24>(0.02f)).frac();
                        Leds::led_buffer[c] = rgb(hsv(h, fixed32<24>(0.5f) + fixed32<24>(0.5f) * fixed32<24>(std::get<2>(Leds::map[c])), fixed32<24>(1.0f) - fixed32<24>(0.9f) * fixed32<24>(std::get<2>(Leds::map[c]))));
                    }
                } break;
        case    4: {
                    for (size_t c = 0; c < Leds::ledsN; c++) {
                        fixed32<24> h = ( fixed32<24>(1.0f) - (fixed32<24>(tick) * fixed32<24>(0.2f)).frac() ) * fixed32<24>(6.28318530718f);
                        fixed32<24> x = (std::get<0>(Leds::map[c]) - fixed32<24>(0.5f)) * cos(h) - (std::get<1>(Leds::map[c]) - fixed32<24>(0.5f)) * sin(h);
                        fixed32<24> hue((x * fixed32<24>(0.5f) + fixed32<24>(0.5f) + h).frac());
                        Leds::led_buffer[c] = rgb(hsv(hue, fixed32<24>(1.0f), fixed32<24>(1.0f)));
                    }
                } break;
        case    5: {
                    for (size_t c = 0; c < Leds::ledsN; c++) {
                        fixed32<24> h = fixed32<24>(1.0f) - (fixed32<24>(tick) * fixed32<24>(0.2f)).frac();
                        fixed32<24> hue((h + fixed32<24>(std::get<0>(Leds::map[c])) * fixed32<24>(std::get<1>(Leds::map[c])) * fixed32<24>(1.0f / 2.0f)).frac());
                        Leds::led_buffer[c] = rgb(hsv(hue, fixed32<24>(1.0f), fixed32<24>(1.0f)));
                    }
                } break;
        case    6: {
                    for (size_t c = 0; c < Leds::ledsN; c++) {
                        fixed32<24> h = fixed32<24>(1.0f) - (fixed32<24>(tick) * fixed32<24>(0.02f)).frac();
                        fixed32<24> hue((h - fixed32<24>(std::get<0>(Leds::map[c])) * fixed32<24>(1.0f / 8.0f)).frac());
                        Leds::led_buffer[c] = rgb(hsv(hue, fixed32<24>(1.0f), fixed32<24>(1.0f)));
                    }
                } break;
        case    7: {
                    for (size_t c = 0; c < Leds::ledsN; c++) {
                        Leds::led_buffer[c] = rgb(fixed32<24>(0.0f), fixed32<24>(0.0f), fixed32<24>(0.0f));
                    }
					for (size_t c = 0; c < 6; c++) {
						int32_t i = rnd.get(0,Leds::ledsN);
						fixed32<24> v = fixed32<24>(1.0f/127.0f)*fixed32<24>(rnd.get(0,127));
						Leds::led_buffer[i] = rgb(v, v, v);
					}
                } break;
        case    8: {
                    for (size_t c = 0; c < Leds::ledsN; c++) {
                        Leds::led_buffer[c] = rgb(fixed32<24>(0.0f), fixed32<24>(0.0f), fixed32<24>(0.0f));
                    }
					for (size_t c = 0; c < 6; c++) {
						int32_t i = rnd.get(0,Leds::ledsN);
						Leds::led_buffer[i] = rgb(
							fixed32<24>(1.0f/127.0f)*fixed32<24>(rnd.get(0,127)), 
							fixed32<24>(1.0f/127.0f)*fixed32<24>(rnd.get(0,127)), 
							fixed32<24>(1.0f/127.0f)*fixed32<24>(rnd.get(0,127)));
					}
                } break;
    }
    tick.raw += 274873;
#ifndef USE_HAL_DRIVER
    printf("\033[0H"); fflush(stdout);    
    for (size_t c = 0; c < Leds::ledsN; c++) {
        printf("\033[%d;%dH\033[48;2;%d;%d;%dm  \033[48;2;0;0;0m",
            16-static_cast<int32_t>(std::get<1>(Leds::map[c]) * fixed32<24>(16)),
            static_cast<int32_t>(std::get<0>(Leds::map[c]) * fixed32<24>(32)),
            int32_t(float(Leds::led_buffer[c].r)*255.0f),
            int32_t(float(Leds::led_buffer[c].g)*255.0f),
            int32_t(float(Leds::led_buffer[c].b)*255.0f));
    }
#endif  // #ifndef USE_HAL_DRIVER

    Leds::instance().transfer();
}

#ifndef USE_HAL_DRIVER
int main() {
	printf("\033[2J"); fflush(stdout);	
	for(;;) {
		HAL_TIM_PeriodElapsedCallback(0);
		std::this_thread::sleep_for(std::chrono::microseconds(16384));
#ifdef WIN32
		if(GetKeyState(VK_SPACE) & 0x8000) {
			while(GetKeyState(VK_SPACE) & 0x8000) {
				std::this_thread::sleep_for(std::chrono::microseconds(1000));
			}
			Model::instance().IncPattern();
		}
#endif // #ifdef WIN32
	}
	return 0;
}
#endif  // #ifndef USE_HAL_DRIVER
