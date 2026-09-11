//实用函数
// Utility functions

#pragma once

#include <math.h>
#ifdef CONFIG_IDF_TARGET_ESP32
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#elif defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
#include "esp_private/brownout.h"
#endif

const float ONE_G = 9.80665;
extern float t;

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool invalid(float x) {
	return !isfinite(x);
}

bool valid(float x) {
	return isfinite(x);
}

// Wrap angle to [-PI, PI)
float wrapAngle(float angle) {
	angle = fmodf(angle, 2 * PI);
	if (angle > PI) {
		angle -= 2 * PI;
	} else if (angle < -PI) {
		angle += 2 * PI;
	}
	return angle;
}

// 通过空格修剪和分割字符串
void disableBrownOut() {
#ifdef CONFIG_IDF_TARGET_ESP32
	REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
#elif defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
	esp_brownout_disable();
#endif
}

// Trim and split string by spaces
void splitString(String& str, String& token0, String& token1, String& token2) {
	str.trim();
	char chars[str.length() + 1];
	str.toCharArray(chars, str.length() + 1);
	token0 = strtok(chars, " ");
	token1 = strtok(NULL, " "); // String(NULL) 创建空字符串
	token2 = strtok(NULL, "");
}

// 速率限制器
class Rate {
public:
	float rate;
	float last = 0;
	Rate(float rate) : rate(rate) {}

	operator bool() {
		if (t - last >= 1 / rate) {
			last = t;
			return true;
		}
		return false;
	}
};

// 布尔信号延迟滤波器——确保信号至少持续“延迟”秒
class Delay {
public:
	float delay;
	float start = NAN;
	Delay(float delay) : delay(delay) {}

	bool update(bool on) {
		if (!on) {
			start = NAN;
			return false;
		} else if (isnan(start)) {
			start = t;
		}
		return t - start >= delay;
	}
};
