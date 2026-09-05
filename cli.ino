// Implementation of command line interface 

#include "pid.h"
#include "vector.h"
#include "util.h"
#include "lpf.h"

extern LowPassFilter<Vector> gyroBiasFilter;

#if WEB_RC_ENABLED
extern bool webConsoleEnabled;
extern void webLog(const char* msg);
#endif

extern const int MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT;
extern const int RAW, ACRO, STAB, AUTO;
extern float t, dt, loopRate;
extern float controlTime;
extern uint16_t channels[16];
extern float controlRoll, controlPitch, controlThrottle, controlYaw, controlMode;
extern float motors[4];
extern int mode;
extern bool armed;

const char* motd =
"CLI命令菜单，输入相应命令，回车后执行:\n"
"help - 帮助\n"
"p - 显示所有参数\n"
"p <name> - 显示指定参数\n"
"p <name> <value> - 设置参数\n"
"p MOT_PIN_FL 14 - 参数设置示例，前左电机引脚为14\n"
"preset - 重置参数存储，设置参数后运行此命令\n"
"mfr, mfl, mrr, mrl - 测试马达 (马达不受算法影响运转，为了安全不要装桨叶！！！)\n"
"ca - 校准陀螺仪加速度计\n"
"ps - 显示pitch/roll/yaw姿态\n"
"cr - 校准RC遥控器\n"
"rc - 显示RC遥控数据\n"
"wifi - 显示WiFi信息\n"
"ap <ssid> <password> - 配置AP模式SSID和密码\n"
"sta <ssid> <password> - 配置STA客户端模式\n"
"raw/stab/acro/auto - 飞行模式设定\n"
"arm - 解锁无人机\n"
"disarm - 锁定无人机\n"
"psq - 显示姿态四元数\n"
"imu - 显示IMU数据\n"
"time - 显示时间信息\n"
"mot - 显示motor输出\n"
"sys - 显示系统info信息\n"
"log [dump] - 打印日志\n"
"reboot - 重启无人机\n"
"reset - 重置无人机\n";

void print(const char* format, ...) {
	// 固定 1000 字节缓冲区 + vsnprintf 会静默截断超长内容（例如开机菜单 motd），且截断后连换行符都可能丢失，
	// 导致后续打印内容拼接到同一行。这里先用栈上小缓冲区尝试格式化，若实际所需长度超过缓冲区，
	// 再按精确所需大小临时用堆内存重新格式化，避免任何长度的内容被静默截断。
	// 此函数无需理会，直接调用即可
	char stackBuf[512];
	va_list args, argsCopy;
	va_start(args, format);
	va_copy(argsCopy, args);
	int needed = vsnprintf(stackBuf, sizeof(stackBuf), format, argsCopy);
	va_end(argsCopy);
	if (needed < 0) needed = 0; // 编码错误兵底，避免负数导致后续内存分配异常

	char *buf = stackBuf;
	bool heapAllocated = false;
	if (needed >= (int)sizeof(stackBuf)) {
		buf = (char*)malloc(needed + 1);
		if (buf) {
			vsnprintf(buf, needed + 1, format, args);
			heapAllocated = true;
		} else {
			buf = stackBuf; // 内存不足兵底：退回已截断的栈缓冲区内容
		}
	}
	va_end(args);

	Serial.print(buf);
#if WIFI_ENABLED
	mavlinkPrint(buf);
#endif
#if WEB_RC_ENABLED
	if (webConsoleEnabled) webLog(buf);
#endif
	if (heapAllocated) free(buf);
}

void pause(float duration) {
	/** @param duration 暂停指定时长，期间持续刷新IMU/姿态和处理输入，保持系统响应，单位为秒 */
	float start = t;
	while (t - start < duration) {
		readIMU(); // 长时间阻塞命令（ca/cr）期间也需要持续刷新IMU/姿态，否则打印信息会定格在进入pause前的旧值
		step();
		estimate();
		handleInput();
#if WIFI_ENABLED
		processMavlink();
#endif
#if WEB_RC_ENABLED
		readWebRC(); // 保持 HTTP 服务器在长时间命令（ca/cr）期间持续响应
#endif
		delay(50);
	}
}

void doCommand(String str, bool echo = false) {
	/** @brief 处理CLI命令
	 *  @param str 输入的命令字符串
	 *  @param echo 是否回显命令
	 */
	// 解析命令字符串
	String command, arg0, arg1;
	splitString(str, command, arg0, arg1);
	if (command.isEmpty()) return;

	// echo command
	if (echo) {
		print("> %s\n", str.c_str());
	}

	command.toLowerCase();

	// 执行命令
	if (command == "help" || command == "motd") {
		/** 打印开机信息 */
		print("%s\n", motd);
	} else if (command == "p" && arg0 == "") {
		/** 打印所有参数 */
		printParameters();
	} else if (command == "p" && arg0 != "" && arg1 == "") {
		/** 打印指定参数的值 */
		print("%s的值为%g\n", arg0.c_str(), getParameter(arg0.c_str()));
	} else if (command == "p") {
		/** 设置指定参数的值 */
		bool success = setParameter(arg0.c_str(), arg1.toFloat());
		if (success) {
			print("%s已成功设置为%g\n", arg0.c_str(), getParameter(arg0.c_str()));
		} else {
			print("未找到参数: '%s', 请执行p命令查看可用参数\n", arg0.c_str());
		}
	} else if (command == "preset") {
		/** 重置所有参数为默认值 */
		resetParameters();
		print("所有参数已重置为默认值\n");
	} else if (command == "time") {
		/** 打印时间信息 */
		print("当前运行时间: %f 秒\n", t);
		print("主循环频率: %.0f 帧\n", loopRate);
		print("最近一帧耗时: %f 秒\n", dt);
	} else if (command == "ps") {
		/** 打印姿态信息 */
		Vector a = attitude.toEuler();
		print("横滚角x: %f  俯仰角y: %f 偏航角z: %f 单位：度\n", degrees(a.x), degrees(a.y), degrees(a.z));
	} else if (command == "psq") {
		/** 打印四元数原始值 */
		print("四元数原始值：qw: %f qx: %f qy: %f qz: %f\n", attitude.w, attitude.x, attitude.y, attitude.z);
	} else if (command == "imu") {
		/** 打印IMU信息 */
		printIMUInfo();
		printIMUCalibration();
		print("落地状态: %d\n", landed);
	} else if (command == "arm") {
		/** 解锁电机 */
		extern bool imuOK;
		if (!imuOK) { print("IMU故障，禁止解锁！\n"); }
		else {armed = true;
		print("电机已解锁\n");
		}
	} else if (command == "disarm") {
		/** 锁定电机 */
		armed = false;
		print("电机已锁定\n");
	} else if (command == "raw") {
		/** 切换为手动模式 */
		mode = RAW;
		print("飞控模式已切换为 手动模式\n");
	} else if (command == "stab") {
		/** 切换为自稳模式 */
		mode = STAB;
		print("飞控模式已切换为 自稳模式\n");
	} else if (command == "acro") {
		/** 切换为特技模式 */
		mode = ACRO;
		print("飞控模式已切换为 特技模式\n");
	} else if (command == "auto") {
		/** 切换为自动模式 */
		mode = AUTO;
		print("飞控模式已切换为自动模式\n");
	} else if (command == "rc") {
		/** 打印遥控器通道信息 */
		print("通道: ");
		for (int i = 0; i < 16; i++) {
			print("%u, ", channels[i]);
		}
		print("\n横滚角x: %g  俯仰角y: %g  偏航角z: %g  油门: %g  模式: %g\n",
			controlRoll, controlPitch, controlYaw, controlThrottle, controlMode);
		print("时间: %.1f\n", controlTime);
		print("飞控模式: %s\n", getModeName());
		print("电机解锁状态: %d\n", armed);
	} else if (command == "wifi") {
		/** 打印WiFi信息 */
#if WIFI_ENABLED
		printWiFiInfo();
#endif
	} else if (command == "ap") {
		/** 配置为WiFi热点模式，arg0为SSID，arg1为密码 */
#if WIFI_ENABLED
		configWiFi(true, arg0.c_str(), arg1.c_str());
#endif
	} else if (command == "sta") {
		/** 配置为WiFi客户端模式，arg0为SSID，arg1为密码 */
#if WIFI_ENABLED
		configWiFi(false, arg0.c_str(), arg1.c_str());
#endif
	} else if (command == "mot") {
		/** 打印电机信息 */
		print("前右 %g 前左 %g 后右 %g 后左 %g\n",
			motors[MOTOR_FRONT_RIGHT], motors[MOTOR_FRONT_LEFT], motors[MOTOR_REAR_RIGHT], motors[MOTOR_REAR_LEFT]);
	} else if (command == "log") {
		/** 打印日志信息 */
		printLogHeader();
		if (arg0 == "dump") printLogData();
	} else if (command == "cr") {
		/** 校准遥控器 */
		calibrateRC();
	} else if (command == "ca") {
		/** 校准加速度计 */
		calibrateAccel();
	} else if (command == "mfr") {
		/** 测试前右电机（马达不受算法影响运转，为了安全不要装桨叶！！！) */
		testMotor(MOTOR_FRONT_RIGHT);
	} else if (command == "mfl") {
		/** 测试前左电机（马达不受算法影响运转，为了安全不要装桨叶！！！） */
		testMotor(MOTOR_FRONT_LEFT);
	} else if (command == "mrr") {
		/** 测试后右电机（马达不受算法影响运转，为了安全不要装桨叶！！！） */
		testMotor(MOTOR_REAR_RIGHT);
	} else if (command == "mrl") {
		/** 测试后左电机（马达不受算法影响运转，为了安全不要装桨叶！！！） */
		testMotor(MOTOR_REAR_LEFT);
	} else if (command == "sys") {
		/** 打印系统信息 */
#ifdef ESP32
		print("芯片: %s\n", ESP.getChipModel());
		print("芯片温度: %.1f °C\n", temperatureRead());
		print("剩余内存: %d\n", ESP.getFreeHeap());
		// Print tasks table
		print("序号  名称               剩余栈  优先级  核心  CPU%%\n");
		int taskCount = uxTaskGetNumberOfTasks();
		TaskStatus_t *systemState = new TaskStatus_t[taskCount];
		uint32_t totalRunTime;
		uxTaskGetSystemState(systemState, taskCount, &totalRunTime);
		for (int i = 0; i < taskCount; i++) {
			String core = systemState[i].xCoreID == tskNO_AFFINITY ? "*" : String(systemState[i].xCoreID);
			int cpuPercentage = systemState[i].ulRunTimeCounter / (totalRunTime / 100);
			print("%-5d%-20s%-7d%-6d%-6s%d\n",systemState[i].xTaskNumber, systemState[i].pcTaskName,
				systemState[i].usStackHighWaterMark, systemState[i].uxCurrentPriority, core.c_str(), cpuPercentage);
		}
		delete[] systemState;
#endif
	} else if (command == "reset") {
		/** 重置姿态 */
		attitude = Quaternion();
		gyroBiasFilter.reset();
	} else if (command == "reboot") {
		/** 软重启飞控系统 */
		ESP.restart();
	} else {
		print("无效命令: '%s', 请执行help命令查看可用命令\n", command.c_str());
	}
}

void handleInput() {
	/** @brief 处理串口输入命令 */
	static bool showMotd = true;
	static String input;

	if (showMotd) {
		print("%s\n", motd);
		showMotd = false;
	}

	while (Serial.available()) {
		char c = Serial.read();
		if (c == '\n') {
			doCommand(input);
			input.clear();
		} else {
			input += c;
		}
	}
}
