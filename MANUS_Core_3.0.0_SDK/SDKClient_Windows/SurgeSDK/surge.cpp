#include "surge.h"
#include <Windows.h> // <WinBase.h> is redundant if you include this
#include <iostream>

//New includes for csv
#include <fstream>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <iostream>
#include <sstream>
#include <sys/stat.h>        // For file size check
#include <locale>
#include <codecvt>           // For wchar_t to UTF-8 conversion

surge_Comm::surge_Comm(std::wstring comm_Port) {
	this->logging = false;
	this->calibrate = false;
	this->pcCommPort = comm_Port.c_str();  // wide string passed directly
	this->hCom = CreateFileW(this->pcCommPort,             // Use CreateFileW explicitly
					   GENERIC_READ | GENERIC_WRITE,
					   0,
					   NULL,
					   OPEN_EXISTING,
					   0,
					   NULL);

	if (this->hCom == INVALID_HANDLE_VALUE) {
		std::wcout << L"CreateFile failed with error " << GetLastError() << L"\n";
		return;
	}

	SecureZeroMemory(&this->dcb, sizeof(DCB));
	this->dcb.DCBlength = sizeof(DCB);

	this->fSuccess = GetCommState(this->hCom, &this->dcb);
	if (!this->fSuccess) {
		std::wcout << L"GetCommState failed with error " << GetLastError() << L"\n";
		return;
	}

	this->dcb.BaudRate = CBR_115200;
	this->dcb.ByteSize = 8;
	this->dcb.Parity = NOPARITY;
	this->dcb.StopBits = ONESTOPBIT;

	this->fSuccess = SetCommState(this->hCom, &this->dcb);
	if (!this->fSuccess) {
		std::wcout << L"SetCommState failed with error " << GetLastError() << L"\n";
		return;
	}

	PrintCommState();

	std::string str = "";
	std::string str_EOL = "";
	utf8_encode(str_EOL, 0x0D);
	std::string message = "AT+SMC=1" + str_EOL;
	//#############SENDING OVER RS485. Raptor, change this to use wifi###################
	LPCVOID lpBuffer = message.c_str();
	DWORD nNumberOfBytesToWrite = strlen(message.c_str());
	LPDWORD lpNumberOfBytesWritten = 0;
	LPOVERLAPPED lpOverlapped = 0;
	printf(message.c_str());
	printf("\n");
	if (!WriteFile(this->hCom, lpBuffer, nNumberOfBytesToWrite, lpNumberOfBytesWritten, lpOverlapped)) {
		printf("SERIAL - Failed to send message Manus Control message\n");
	}
}

void surge_Comm::sendCalibration(float * angles) {

	for (int i = 0; i < 12; i++) {
		if (angles[i] > 127.0) {
			angles[i] = 127.0;
		}
		if (angles[i] < 0.0) {
			angles[i] = 0.0;
		}
	}

	//min
	uint8_t whole_angle_TR_min = static_cast<uint8_t>(angles[0]);
	uint8_t whole_angle_TF_min = static_cast<uint8_t>(angles[1]);
	uint8_t whole_angle_I_min = static_cast<uint8_t>(angles[2]);
	uint8_t whole_angle_M_min = static_cast<uint8_t>(angles[3]);
	uint8_t whole_angle_R_min = static_cast<uint8_t>(angles[4]);
	uint8_t whole_angle_P_min = static_cast<uint8_t>(angles[5]);
	//max
	uint8_t whole_angle_TR_max = static_cast<uint8_t>(angles[6]);
	uint8_t whole_angle_TF_max = static_cast<uint8_t>(angles[7]);
	uint8_t whole_angle_I_max = static_cast<uint8_t>(angles[8]);
	uint8_t whole_angle_M_max = static_cast<uint8_t>(angles[9]);
	uint8_t whole_angle_R_max = static_cast<uint8_t>(angles[10]);
	uint8_t whole_angle_P_max = static_cast<uint8_t>(angles[11]);
	//min
	std::string str_TF_min = "";
	std::string str_TR_min = "";
	std::string str_I_min = "";
	std::string str_M_min = "";
	std::string str_R_min = "";
	std::string str_P_min = "";
	std::string str_EOL = "";
	//max
	std::string str_TF_max = "";
	std::string str_TR_max = "";
	std::string str_I_max = "";
	std::string str_M_max = "";
	std::string str_R_max = "";
	std::string str_P_max = "";
	utf8_encode(str_EOL, 0x0D);
	//min
	/*
	utf8_encode(str_TF_min, whole_angle_TF_min);
	utf8_encode(str_TR_min, whole_angle_TR_min);
	utf8_encode(str_I_min, whole_angle_I_min);
	utf8_encode(str_M_min, whole_angle_M_min);
	utf8_encode(str_R_min, whole_angle_R_min);
	utf8_encode(str_P_min, whole_angle_P_min);
	
	//max
	utf8_encode(str_TF_max, whole_angle_TF_max);
	utf8_encode(str_TR_max, whole_angle_TR_max);
	utf8_encode(str_I_max, whole_angle_I_max);
	utf8_encode(str_M_max, whole_angle_M_max);
	utf8_encode(str_R_max, whole_angle_R_max);
	utf8_encode(str_P_max, whole_angle_P_max);
	//min (L = low)
	std::string message_TF_min = "AT+SCL=" + str_TF_min + ",TF" + str_EOL;
	std::string message_TR_min = "AT+SCL=" + str_TR_min + ",TR" + str_EOL;
	std::string message_I_min = "AT+SCL=" + str_I_min + ",I" + str_EOL;
	std::string message_M_min = "AT+SCL=" + str_M_min + ",M" + str_EOL;
	std::string message_R_min = "AT+SCL=" + str_R_min + ",R" + str_EOL;
	std::string message_P_min = "AT+SCL=" + str_P_min + ",P" + str_EOL;
	//max (H = high)
	std::string message_TF_max = "AT+SCH=" + str_TF_max + ",TF" + str_EOL;
	std::string message_TR_max = "AT+SCH=" + str_TR_max + ",TR" + str_EOL;
	std::string message_I_max = "AT+SCH=" + str_I_max + ",I" + str_EOL;
	std::string message_M_max = "AT+SCH=" + str_M_max + ",M" + str_EOL;
	std::string message_R_max = "AT+SCH=" + str_R_max + ",R" + str_EOL;
	std::string message_P_max = "AT+SCH=" + str_P_max + ",P" + str_EOL;*/
	//min (L = low)
	std::string message_TF_min = "AT+SCL=" + std::to_string(whole_angle_TF_min) + ",TF" + str_EOL;
	std::string message_TR_min = "AT+SCL=" + std::to_string(whole_angle_TR_min) + ",TR" + str_EOL;
	std::string message_I_min = "AT+SCL=" + std::to_string(whole_angle_I_min) + ",I" + str_EOL;
	std::string message_M_min = "AT+SCL=" + std::to_string(whole_angle_M_min) + ",M" + str_EOL;
	std::string message_R_min = "AT+SCL=" + std::to_string(whole_angle_R_min) + ",R" + str_EOL;
	std::string message_P_min = "AT+SCL=" + std::to_string(whole_angle_P_min) + ",P" + str_EOL;
	//max (H = high)
	std::string message_TF_max = "AT+SCH=" + std::to_string(whole_angle_TF_max) + ",TF" + str_EOL;
	std::string message_TR_max = "AT+SCH=" + std::to_string(whole_angle_TR_max) + ",TR" + str_EOL;
	std::string message_I_max = "AT+SCH=" + std::to_string(whole_angle_I_max) + ",I" + str_EOL;
	std::string message_M_max = "AT+SCH=" + std::to_string(whole_angle_M_max) + ",M" + str_EOL;
	std::string message_R_max = "AT+SCH=" + std::to_string(whole_angle_R_max) + ",R" + str_EOL;
	std::string message_P_max = "AT+SCH=" + std::to_string(whole_angle_P_max) + ",P" + str_EOL;

	//std::string message = "AT+SJA=" + std::to_string((angle)) + "," + finger + str_EOL;

	//###################### TIMESTAMP (with milliseconds) ########################
	auto now = std::chrono::system_clock::now();
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		now.time_since_epoch()
	) % 1000;

	std::time_t now_c = std::chrono::system_clock::to_time_t(now);
	std::tm now_tm;
#if defined(_WIN32) || defined(_WIN64)
	localtime_s(&now_tm, &now_c);
#else
	localtime_r(&now_c, &now_tm);
#endif

	std::ostringstream timestamp;
	timestamp << std::put_time(&now_tm, "%H:%M:%S")
		<< "." << std::setw(3) << std::setfill('0') << ms.count();

	//###################### PRINT TO CONSOLE ########################
	//min
	std::cout << "[" << timestamp.str() << "] " << message_TF_min << std::endl;
	std::cout << "[" << timestamp.str() << "] " << message_TR_min << std::endl;
	std::cout << "[" << timestamp.str() << "] " << message_I_min << std::endl;
	std::cout << "[" << timestamp.str() << "] " << message_M_min << std::endl;
	std::cout << "[" << timestamp.str() << "] " << message_R_min << std::endl;
	std::cout << "[" << timestamp.str() << "] " << message_P_min << std::endl;
	//max
	std::cout << "[" << timestamp.str() << "] " << message_TF_max << std::endl;
	std::cout << "[" << timestamp.str() << "] " << message_TR_max << std::endl;
	std::cout << "[" << timestamp.str() << "] " << message_I_max << std::endl;
	std::cout << "[" << timestamp.str() << "] " << message_M_max << std::endl;
	std::cout << "[" << timestamp.str() << "] " << message_R_max << std::endl;
	std::cout << "[" << timestamp.str() << "] " << message_P_max << std::endl;

	//###################### SEND OVER SERIAL ########################
	//min
	LPCVOID lpBuffer = message_TF_min.c_str();
	DWORD nNumberOfBytesToWrite = strlen(message_TF_min.c_str());
	LPDWORD lpNumberOfBytesWritten = 0;
	LPOVERLAPPED lpOverlapped = nullptr;

	if (!WriteFile(this->hCom, lpBuffer, nNumberOfBytesToWrite, lpNumberOfBytesWritten, lpOverlapped)) {
		printf("SERIAL - Failed to send message\n");
	}

	lpBuffer = message_TR_min.c_str();
	nNumberOfBytesToWrite = strlen(message_TR_min.c_str());
	lpNumberOfBytesWritten = 0;
	lpOverlapped = nullptr;

	if (!WriteFile(this->hCom, lpBuffer, nNumberOfBytesToWrite, lpNumberOfBytesWritten, lpOverlapped)) {
		printf("SERIAL - Failed to send message\n");
	}

	lpBuffer = message_I_min.c_str();
	nNumberOfBytesToWrite = strlen(message_I_min.c_str());
	lpNumberOfBytesWritten = 0;
	lpOverlapped = nullptr;

	if (!WriteFile(this->hCom, lpBuffer, nNumberOfBytesToWrite, lpNumberOfBytesWritten, lpOverlapped)) {
		printf("SERIAL - Failed to send message\n");
	}

	lpBuffer = message_M_min.c_str();
	nNumberOfBytesToWrite = strlen(message_M_min.c_str());
	lpNumberOfBytesWritten = 0;
	lpOverlapped = nullptr;

	if (!WriteFile(this->hCom, lpBuffer, nNumberOfBytesToWrite, lpNumberOfBytesWritten, lpOverlapped)) {
		printf("SERIAL - Failed to send message\n");
	}

	lpBuffer = message_R_min.c_str();
	nNumberOfBytesToWrite = strlen(message_R_min.c_str());
	lpNumberOfBytesWritten = 0;
	lpOverlapped = nullptr;

	if (!WriteFile(this->hCom, lpBuffer, nNumberOfBytesToWrite, lpNumberOfBytesWritten, lpOverlapped)) {
		printf("SERIAL - Failed to send message\n");
	}

	lpBuffer = message_P_min.c_str();
	nNumberOfBytesToWrite = strlen(message_P_min.c_str());
	lpNumberOfBytesWritten = 0;
	lpOverlapped = nullptr;

	if (!WriteFile(this->hCom, lpBuffer, nNumberOfBytesToWrite, lpNumberOfBytesWritten, lpOverlapped)) {
		printf("SERIAL - Failed to send message\n");
	}

	//max
	lpBuffer = message_TF_max.c_str();
	nNumberOfBytesToWrite = strlen(message_TF_max.c_str());
	lpNumberOfBytesWritten = 0;
	lpOverlapped = nullptr;

	if (!WriteFile(this->hCom, lpBuffer, nNumberOfBytesToWrite, lpNumberOfBytesWritten, lpOverlapped)) {
		printf("SERIAL - Failed to send message\n");
	}

	lpBuffer = message_TR_max.c_str();
	nNumberOfBytesToWrite = strlen(message_TR_max.c_str());
	lpNumberOfBytesWritten = 0;
	lpOverlapped = nullptr;

	if (!WriteFile(this->hCom, lpBuffer, nNumberOfBytesToWrite, lpNumberOfBytesWritten, lpOverlapped)) {
		printf("SERIAL - Failed to send message\n");
	}

	lpBuffer = message_I_max.c_str();
	nNumberOfBytesToWrite = strlen(message_I_max.c_str());
	lpNumberOfBytesWritten = 0;
	lpOverlapped = nullptr;

	if (!WriteFile(this->hCom, lpBuffer, nNumberOfBytesToWrite, lpNumberOfBytesWritten, lpOverlapped)) {
		printf("SERIAL - Failed to send message\n");
	}

	lpBuffer = message_M_max.c_str();
	nNumberOfBytesToWrite = strlen(message_M_max.c_str());
	lpNumberOfBytesWritten = 0;
	lpOverlapped = nullptr;

	if (!WriteFile(this->hCom, lpBuffer, nNumberOfBytesToWrite, lpNumberOfBytesWritten, lpOverlapped)) {
		printf("SERIAL - Failed to send message\n");
	}

	lpBuffer = message_R_max.c_str();
	nNumberOfBytesToWrite = strlen(message_R_max.c_str());
	lpNumberOfBytesWritten = 0;
	lpOverlapped = nullptr;

	if (!WriteFile(this->hCom, lpBuffer, nNumberOfBytesToWrite, lpNumberOfBytesWritten, lpOverlapped)) {
		printf("SERIAL - Failed to send message\n");
	}

	lpBuffer = message_P_max.c_str();
	nNumberOfBytesToWrite = strlen(message_P_max.c_str());
	lpNumberOfBytesWritten = 0;
	lpOverlapped = nullptr;

	if (!WriteFile(this->hCom, lpBuffer, nNumberOfBytesToWrite, lpNumberOfBytesWritten, lpOverlapped)) {
		printf("SERIAL - Failed to send message\n");
	}

}

void surge_Comm::sendString(std::string finger, float angle, bool hand) {
	if (angle > 127.0) {
		angle = 127.0;
	}
	if (angle < 0.0) {
		angle = 0.0;
	}
	uint8_t whole_angle = static_cast<uint8_t>(angle);

	std::string str = "";
	std::string str_EOL = "";
	utf8_encode(str, whole_angle);
	utf8_encode(str_EOL, 0x0D);

	std::ostringstream oss;
	oss << std::fixed << std::setprecision(1) << angle;
	std::string message = "AT+SJA=" + oss.str() + "," + finger + str_EOL;

	//std::string message = "AT+SJA=" + std::to_string((angle)) + "," + finger + str_EOL;

	//###################### TIMESTAMP (with milliseconds) ########################
	auto now = std::chrono::system_clock::now();
	auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		now.time_since_epoch()
	) % 1000;

	std::time_t now_c = std::chrono::system_clock::to_time_t(now);
	std::tm now_tm;
#if defined(_WIN32) || defined(_WIN64)
	localtime_s(&now_tm, &now_c);
#else
	localtime_r(&now_c, &now_tm);
#endif

	std::ostringstream timestamp;
	timestamp << std::put_time(&now_tm, "%H:%M:%S")
		<< "." << std::setw(3) << std::setfill('0') << ms.count();

	//###################### PRINT TO CONSOLE ########################
	std::cout << "[" << timestamp.str() << "] " << message << std::endl;

	//###################### SEND OVER SERIAL ########################
	LPCVOID lpBuffer = message.c_str();
	DWORD nNumberOfBytesToWrite = strlen(message.c_str());
	LPDWORD lpNumberOfBytesWritten = 0;
	LPOVERLAPPED lpOverlapped = nullptr;

	if (!WriteFile(this->hCom, lpBuffer, nNumberOfBytesToWrite, lpNumberOfBytesWritten, lpOverlapped)) {
		printf("SERIAL - Failed to send message\n");
	}

	//###################### LOG TO CSV ########################
	if (this->logging == true) {
		try {
			const char* filename = "log.csv";
			bool fileExistsAndEmpty = false;

			struct stat fileStat;
			if (stat(filename, &fileStat) == 0) {
				fileExistsAndEmpty = (fileStat.st_size == 0);
			}
			else {
				fileExistsAndEmpty = true; // File doesn't exist
			}

			std::ofstream logFile(filename, std::ios::app); // Append mode
			if (logFile.is_open()) {
				if (fileExistsAndEmpty) {
					logFile << "time,com,angle,finger" << std::endl;
				}

				// Convert COM port from wchar_t* to std::string (UTF-8)
				std::wstring ws(this->pcCommPort);
				std::wstring_convert<std::codecvt_utf8<wchar_t>> conv;
				std::string comPortUtf8 = conv.to_bytes(ws);

				logFile << timestamp.str() << ","
					<< comPortUtf8 << ","
					<< static_cast<int>(whole_angle) << ","
					<< finger << std::endl;

				logFile.flush();
			}
		}
		catch (...) {
			printf("LOGGING - Failed to write to log.csv\n");
		}
	}
}
/*
void surge_Comm::sendString(std::string finger, float angle, bool hand) {
	uint8_t whole_angle = (uint8_t)angle;
	if (whole_angle > 127) {
		whole_angle = 127;
	}
	std::string str = "";
	std::string str_EOL = "";
	utf8_encode(str, whole_angle);
	utf8_encode(str_EOL, 0x0D);
	std::string message = "AT+SJA=" + std::to_string((char)whole_angle) + "," + finger + str_EOL;
	//#############SENDING OVER RS485. Raptor, change this to use wifi###################
	LPCVOID lpBuffer = message.c_str();
	DWORD nNumberOfBytesToWrite = strlen(message.c_str());
	LPDWORD lpNumberOfBytesWritten = 0;
	LPOVERLAPPED lpOverlapped = 0;
	printf(message.c_str());
	printf("\n");
	if (!WriteFile(this->hCom, lpBuffer, nNumberOfBytesToWrite, lpNumberOfBytesWritten, lpOverlapped)) {
		printf("SERIAL - Failed to send message\n");
	}
}
*/
void surge_Comm::utf8_encode(std::string& str, unsigned long code_point) {
	if (code_point <= 0x007F) {
		char ch = static_cast<char>(code_point);

		str.push_back(ch);
	}
	else if (code_point <= 0x07FF) {
		//Use uint8_t or unsigned char. Regular char may be signed
		//and the sign bit may cause defect during bit manipulation
		uint8_t b2 = 0b10000000 | (code_point & 0b111111);
		uint8_t b1 = 0b11000000 | (code_point >> 6);

		str.push_back(b1);
		str.push_back(b2);
	}
	else if (code_point <= 0xFFFF) {
		uint8_t b3 = 0b10000000 | (code_point & 0b111111);
		uint8_t b2 = 0b10000000 | ((code_point >> 6) & 0b111111);
		uint8_t b1 = 0b11100000 | (code_point >> 12);

		str.push_back(b1);
		str.push_back(b2);
		str.push_back(b3);
	}
	else if (code_point <= 0x10FFFF) {
		uint8_t b4 = 0b10000000 | (code_point & 0b111111);
		uint8_t b3 = 0b10000000 | ((code_point >> 6) & 0b111111);
		uint8_t b2 = 0b10000000 | ((code_point >> 12) & 0b111111);
		uint8_t b1 = 0b11110000 | (code_point >> 18);

		str.push_back(b1);
		str.push_back(b2);
		str.push_back(b3);
		str.push_back(b4);
	}
}

void surge_Comm::PrintCommState(void) {
	//  Print some of the DCB structure values
	printf("\nBaudRate = %d, ByteSize = %d, Parity = %d, StopBits = %d\n",
			  dcb.BaudRate,
			  dcb.ByteSize,
			  dcb.Parity,
			  dcb.StopBits);
}