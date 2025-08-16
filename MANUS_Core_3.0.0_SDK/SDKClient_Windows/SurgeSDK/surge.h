#ifndef __SURGE_H__
#define __SURGE_H__

#include <string>
#define NOMINMAX 
#include <Windows.h>
class surge_Comm {
public:
    surge_Comm() = default;
    surge_Comm(std::wstring comm_Port);  //Use std::wstring for Unicode
    void sendString(std::string finger, float angle, bool hand);
    void sendCalibration(float* angles);
    BOOL logging;
    BOOL calibrate;
private:
    void utf8_encode(std::string& str, unsigned long code_point);
    void PrintCommState(void);
    DCB dcb;
    LPCWSTR pcCommPort; // Now clearly wide char pointer
    HANDLE hCom;
    BOOL fSuccess;
    
};

#endif