//  g++ -o hello .\firsttest.cpp .\picfunc.cpp 

#include <stdio.h>
#include <windows.h>
#include <pthread.h>
#include "picfunc.h"

int main()
{
    HINSTANCE hModule = LoadLibraryA("NMCLIB04v64.dll");
    Func_NmcInit NmcInit = LoadNmcInit(hModule);
    Func_NmcGetModType NmcGetModType = LoadNmcGetModType(hModule);
    Func_NmcNoOp NmcNoOp = LoadNmcNoOp(hModule);
    Func_NmcGetStat NmcGetStat = LoadNmcGetStat(hModule);
    Func_NmcShutdown NmcShutdown = LoadNmcShutdown(hModule);
    Func_ServoResetPos ServoResetPos = LoadServoResetPos(hModule);
    Func_ServoLoadTraj ServoLoadTraj = LoadServoLoadTraj(hModule);
    Func_ServoStopMotor ServoStopMotor = LoadServoStopMotor(hModule);
    Func_ServoSetGain ServoSetGain = LoadServoSetGain(hModule);

    int numberOfModules = NmcInit("COM3:", 19200);
    printf("Number of Modules detected: %d\n", numberOfModules);
    if (numberOfModules == 0){
        printf("failed to initialize. Shutdown.");
        NmcShutdown();
        FreeLibrary(hModule);
        printf("Done.\n");
    }

    if (NmcGetModType(3) == SERVOMODTYPE)
    {
        printf("OK\n");
    }

    for (int i = 1; i <= 2; i ++)
    {
        ServoStopMotor(i, AMP_ENABLE | MOTOR_OFF);   // enable amp
        ServoStopMotor(i, AMP_ENABLE | STOP_ABRUPT); // stop at current pos.
        ServoResetPos(i);                            // reset the posiiton counter to 0

        ServoSetGain(i, // axis = 1
                     100,  // Kp = 100
                     1000, // Kd = 1000
                     0,    // Ki = 0
                     0,    // IL = 0
                     255,  // OL = 255
                     0,    // CL = 0
                     4000, // EL = 4000
                     1,    // SR = 1
                     0     // DC = 0
        );
    }

    for (int i = 1; i <= 2; i++)
    {
        BOOL success = ServoLoadTraj(i, // addr = 2
                                     LOAD_POS | VEL_MODE| LOAD_VEL | LOAD_ACC | ENABLE_SERVO | START_NOW,
                                     5000,  // pos = 2000
                                     100000, // vel = 100,000
                                     1000,   // acc = 100
                                     0       // pwm = 0
        );

        printf("%d\n", success);
    }

    Sleep(2000);

    for (int i = 1; i <= 2 ; i++)
    {
        // ServoStopMotor(i, AMP_ENABLE | MOTOR_OFF);   // enable amp
        // ServoStopMotor(i, AMP_ENABLE | STOP_ABRUPT); // stop at current pos.
        // ServoResetPos(i);                            // reset the posiiton counter to 0

        // ServoSetGain(i, // axis = 1
        //              100,  // Kp = 100
        //              1000, // Kd = 1000
        //              0,    // Ki = 0
        //              0,    // IL = 0
        //              255,  // OL = 255
        //              0,    // CL = 0
        //              4000, // EL = 4000
        //              1,    // SR = 1
        //              0     // DC = 0
        // );

        BOOL success = ServoLoadTraj(i, // addr = 2
                                     LOAD_POS | VEL_MODE|LOAD_VEL | LOAD_ACC | ENABLE_SERVO | START_NOW,
                                     5000,  // pos = 2000
                                     0, // vel = 100,000
                                     1000,   // acc = 100
                                     0       // pwm = 0
        );
        printf("%d\n", success);
    }

    Sleep(2000);
    
    for (int i = 1; i <= 2; i++)
    {
        BOOL success = ServoLoadTraj(i, // addr = 2
                                     LOAD_POS | VEL_MODE| LOAD_VEL | LOAD_ACC | ENABLE_SERVO | START_NOW,
                                     5000,  // pos = 2000
                                     -100000, // vel = 100,000
                                     1000,   // acc = 100
                                     0       // pwm = 0
        );

        printf("%d\n", success);
    }
    Sleep(2000);

     for (int i = 1; i <= 2 ; i++)
    {
        // ServoStopMotor(i, AMP_ENABLE | MOTOR_OFF);   // enable amp
        // ServoStopMotor(i, AMP_ENABLE | STOP_ABRUPT); // stop at current pos.
        // ServoResetPos(i);                            // reset the posiiton counter to 0

        // ServoSetGain(i, // axis = 1
        //              100,  // Kp = 100
        //              1000, // Kd = 1000
        //              0,    // Ki = 0
        //              0,    // IL = 0
        //              255,  // OL = 255
        //              0,    // CL = 0
        //              4000, // EL = 4000
        //              1,    // SR = 1
        //              0     // DC = 0
        // );

        BOOL success = ServoLoadTraj(i, // addr = 2
                                     LOAD_POS | VEL_MODE|LOAD_VEL | LOAD_ACC | ENABLE_SERVO | START_NOW,
                                     5000,  // pos = 2000
                                     0, // vel = 100,000
                                     1000,   // acc = 100
                                     0       // pwm = 0
        );
        printf("%d\n", success);
    }


    NmcShutdown();

    FreeLibrary(hModule);
    printf("Done.\n");
}
