
#include <stdint.h>
#include <memory.h>
#include <algorithm>

#ifdef USE_HAL_DRIVER
#include "stm32l0xx_hal.h"
extern "C" SPI_HandleTypeDef hspi1;
extern "C" DMA_HandleTypeDef hdma_spi1_tx;
#else  // #ifdef USE_HAL_DRIVER
struct TIM_HandleTypeDef;
#include <stdio.h>
#include <unistd.h>
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
    constexpr inline explicit fixed32(T a) noexcept {
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

	constexpr fixed32 abs() const {
        fixed32 ret; 
		int32_t mask = raw >> 31;
		ret.raw = ( mask ^ raw ) - mask;
		return ret;
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

class hsv;

class rgb {
public:
	fixed32<24> r;
	fixed32<24> g;
	fixed32<24> b;

	constexpr rgb() :
		r(0.0f),
		g(0.0f),
		b(0.0f){
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
		h(0.0f),
		s(0.0f),
		v(0.0f) {
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

class Leds {
public:
    static constexpr size_t ledsN = 8;
    static constexpr size_t spiPaddingBytes = 64;

	static const fixed32<24> map[16];

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

const fixed32<24> Leds::map[16] = {
	fixed32<24>(0.000000000000f),  fixed32<24>(0.000000000000f),
	fixed32<24>(0.086696349084f),  fixed32<24>(0.336206883192f),
	fixed32<24>(0.238528773189f),  fixed32<24>(0.612068951130f),
	fixed32<24>(0.314136117697f),  fixed32<24>(0.232758626342f),
	fixed32<24>(0.638743460178f),  fixed32<24>(0.435344815254f),
	fixed32<24>(1.000000000000f),  fixed32<24>(0.625000000000f),
	fixed32<24>(0.916230380535f),  fixed32<24>(1.000000000000f),
	fixed32<24>(0.371727734804f),  fixed32<24>(0.995689630508f)
};

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
            auto convert_half_to_spi = [] (uint8_t x) {
                return 0x88888888 | (((x >>  4) | (x <<  6) | (x << 16) | (x << 26)) & 0x04040404)|
                                    (((x >>  1) | (x <<  9) | (x << 19) | (x << 29)) & 0x40404040);
            };
            *p++ = convert_half_to_spi((v>>8)&0xFF);
            *p++ = convert_half_to_spi((v>>0)&0xFF);
            return p;
        };
        ptr = convert_to_one_wire_spi(ptr, ((led_buffer[c].r.raw-1)>>8)&0xFFFF);
        ptr = convert_to_one_wire_spi(ptr, ((led_buffer[c].g.raw-1)>>8)&0xFFFF);
        ptr = convert_to_one_wire_spi(ptr, ((led_buffer[c].b.raw-1)>>8)&0xFFFF);
    }

    for (size_t c = 0; c < spiPaddingBytes/(sizeof(uint32_t)*2); c++ ) {
        *ptr++ = 0;
    };

#ifdef USE_HAL_DRIVER
    HAL_SPI_DMAStop(&hspi1);
    HAL_SPI_Transmit_DMA(&hspi1, spi_buffer, sizeof(spi_buffer));
#endif  //#ifdef USE_HAL_DRIVER
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *) {
    static fixed32<24> tick;
    for (size_t c = 0; c < Leds::ledsN; c++) {
        fixed32<24> h = fixed32<24>(1.0f) - (fixed32<24>(tick) * fixed32<24>(0.08f)).frac();
        fixed32<24> hue((h + fixed32<24>(Leds::map[c*2+0]) * fixed32<24>(1.0f / 2.0f)).frac());
        Leds::led_buffer[c] = rgb(hsv(hue, fixed32<24>(1.0f), fixed32<24>(1.0f)));
    }
    tick.raw += 274873;
#ifndef USE_HAL_DRIVER
	printf("\033[0H"); fflush(stdout);	
    for (size_t c = 0; c < Leds::ledsN; c++) {
		printf("\033[%d;%dH\033[48;2;%d;%d;%dm  \033[48;2;0;0;0m",
			16-static_cast<int32_t>(Leds::map[c*2+0] * fixed32<24>(16)),
			static_cast<int32_t>(Leds::map[c*2+1] * fixed32<24>(32)),
			int32_t(float(Leds::led_buffer[c].r)*255.0f),
			int32_t(float(Leds::led_buffer[c].g)*255.0f),
			int32_t(float(Leds::led_buffer[c].b)*255.0f));
	}
#endif  // #ifndef USE_HAL_DRIVER

    Leds::instance().transfer();
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
	HAL_FLASHEx_DATAEEPROM_Erase(0);
	HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, 0, pattern);
	HAL_FLASHEx_DATAEEPROM_Lock();
#endif  // #ifdef USE_HAL_DRIVER
}

void Model::init() {
	load();
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t) {
	Model::instance().IncPattern();
	Model::instance().save();
}

#ifndef USE_HAL_DRIVER
int main() {
	printf("\033[2J"); fflush(stdout);	
	for(;;) {
		HAL_TIM_PeriodElapsedCallback(0);
		usleep(16384);
	}
	return 0;
}
#endif  // #ifndef USE_HAL_DRIVER
