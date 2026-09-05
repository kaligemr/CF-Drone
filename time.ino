// 时间相关函数
// Time related functions

float loopRate; // Hz

void step() {
	/** @brief 更新时间步长并计算循环频率 */
	float now = micros() / 1000000.0;
	dt = now - t;
	t = now;

	if (!(dt > 0)) {
		dt = 0; // assume dt to be zero on first step and on reset
	}

	computeLoopRate();
}

void computeLoopRate() {
	/** @brief 计算循环频率，基于1秒窗口 */
	static float windowStart = 0;
	static uint32_t rate = 0;
	rate++;
	if (t - windowStart >= 1) { // 1 second window
		loopRate = rate;
		windowStart = t;
		rate = 0;
	}
}
