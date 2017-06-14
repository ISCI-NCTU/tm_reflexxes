/*********************************************************************
 *          GNU LESSER GENERAL PUBLIC LICENSE
 *              Version 3, 29 June 2007
 *
 *Copyright (c) 2017, ISCI / National Chiao Tung University (NCTU)
 *
 *Author :  Howard Chen, howardchen.ece04g@g2.nctu.edu.tw
 *
 *This version of the GNU Lesser General Public License incorporates
 *the terms and conditions of version 3 of the GNU General Public License.

 *The Type II Reflexxes Motion Library is free software: you can redistribute
 *it and/or modify it under the terms of the GNU Lesser General Public License
 *as published by the Free Software Foundation, either version 3 of the
 *License, or (at your option) any later version.

 *The Type II Reflexxes Motion Library is distributed in the hope that it
 *will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 *the GNU Lesser General Public License for more details.
 *********************************************************************/
/*********************************************************************
 *                      Apache License
 *                 Version 2.0, January 2004
 *               http://www.apache.org/licenses/
 *
 * tm_modern_driver.cpp
 *
 * Copyright (c) 2017, ISCI / National Chiao Tung University (NCTU)
 *
 * Author: Howard Chen (s880367@gmail.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 **********************************************************************/

/*******************************
TODO :

** Modullize ReflexxesPositioinRun()
2. Position use cartesian position(import T).

** Create ReflexxesVelocityRun()
2. Velocity use cartesian space(import jacobian)

********************************/

#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <sys/time.h>
#include <iostream>
#include <termios.h>
#include <math.h>
#include <iostream>

#include "tm_driver/include/tm_driver/tm_driver.h"

#include "tm_reflexxes/include/tm_reflexxes/tm_reflexxes.h"

#include "ReflexxesTypeII/include/ReflexxesAPI.h"
#include "ReflexxesTypeII/include/RMLPositionFlags.h"
#include "ReflexxesTypeII/include/RMLPositionInputParameters.h"
#include "ReflexxesTypeII/include/RMLPositionOutputParameters.h"

#include "ReflexxesTypeII/include/RMLVelocityFlags.h"
#include "ReflexxesTypeII/include/RMLVelocityInputParameters.h"
#include "ReflexxesTypeII/include/RMLVelocityOutputParameters.h"

//*************************************************************************
// defines

#define CYCLE_TIME_IN_SECONDS                   0.025
#define NUMBER_OF_DOFS                          6
#define DEG2RAD 0.01745329252
#define RAD2DEG 57.29577951

#define MAX_VELOCITY 1.0
#define MAX_ACC 0.0375*40 // 0.0375 : acc in 25ms

using namespace std;


/* tm_moder_driver.cpp */
    /* Initialize new terminal i/o settings */
    void initTermios(int echo)
    {
        tcgetattr(STDIN_FILENO, &oldt); /* grab old terminal i/o settings */
        newt = oldt; /* make new settings same as old settings */
        newt.c_lflag &= ~ICANON; /* disable buffered i/o */
        newt.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
        tcsetattr(STDIN_FILENO, TCSANOW, &newt); /* use these new terminal i/o settings now */
    }
    /* Restore old terminal i/o settings */
    void resetTermios()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
    int kbhit()
    {
        struct timeval tv;
        fd_set rdfs;
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        FD_ZERO(&rdfs);
        FD_SET (STDIN_FILENO, &rdfs);
        select(STDIN_FILENO + 1, &rdfs, NULL, NULL, &tv);
        return FD_ISSET(STDIN_FILENO, &rdfs);
    }

    void print_vectord(const std::vector<double>& vec)
    {
        for (int i = 0; i < vec.size() - 1; i++)
        {
            printf("%.4f, ", vec[i]);
        }
        printf("%.4f", vec[vec.size() - 1]);
    }


    void print_rt_1(const TmDriver& TR, double& time_s, std::vector<double>& vec)
    {
        time_s = TR.interface->stateRT->getQAct(vec);
        printf("[ INFO]  q_act:=< ");
        print_vectord(vec);
        printf(" > [%.3f]\n", time_s);
        time_s = TR.interface->stateRT->getQCmd(vec);
        printf("[ INFO]  q_cmd:=< ");
        print_vectord(vec);
        printf(" > [%.3f]\n", time_s);
    }
    void print_rt_2(const TmDriver& TR, double& time_s, std::vector<double>& vec)
    {
        time_s = TR.interface->stateRT->getQdAct(vec);
        printf("[ INFO] qd_act:=< ");
        print_vectord(vec);
        printf(" > [%.3f]\n", time_s);
        time_s = TR.interface->stateRT->getQdCmd(vec);
        printf("[ INFO] qd_cmd:=< ");
        print_vectord(vec);
        printf(" > [%.3f]\n", time_s);
    }
    void print_rt_3(const TmDriver& TR, double& time_s, std::vector<double>& vec)
    {
        time_s = TR.interface->stateRT->getQtAct(vec);
        printf("[ INFO] qt_act:=< ");
        print_vectord(vec);
        printf(" > [%.3f]\n", time_s);
        time_s = TR.interface->stateRT->getQtCmd(vec);
        printf("[ INFO] qt_cmd:=< ");
        print_vectord(vec);
        printf(" > [%.3f]\n", time_s);
    }
    void print_rt_4(const TmDriver& TR, double& time_s, std::vector<double>& vec)
    {
        time_s = TR.interface->stateRT->getTool0PosAct(vec);
        printf("[ INFO] tool0_pos_act:=< ");
        print_vectord(vec);
        printf(" > [%.3f]\n", time_s);
        time_s = TR.interface->stateRT->getTool0VelAct(vec);
        printf("[ INFO] tool0_vel_act:=< ");
        print_vectord(vec);
        printf(" > [%.3f]\n", time_s);
    }
    void print_rt_5(const TmDriver& TR, double& time_s, std::vector<double>& vec)
    {
        time_s = TR.interface->stateRT->getToolPosAct(vec);
        printf("[ INFO]  tool_pos_act:=< ");
        print_vectord(vec);
        printf(" > [%.3f]\n", time_s);
        time_s = TR.interface->stateRT->getToolPosCmd(vec);
        printf("[ INFO]  tool_pos_cmd:=< ");
        print_vectord(vec);
        printf(" > [%.3f]\n", time_s);
    }
    void print_rt_6(const TmDriver& TR, double& time_s, std::vector<double>& vec)
    {
        time_s = TR.interface->stateRT->getToolVelAct(vec);
        printf("[ INFO]  tool_vel_act:=< ");
        print_vectord(vec);
        printf(" > [%.3f]\n", time_s);
        time_s = TR.interface->stateRT->getToolVelCmd(vec);
        printf("[ INFO]  tool_vel_cmd:=< ");
        print_vectord(vec);
        printf(" > [%.3f]\n", time_s);
    }
    void print_rt_7(const TmDriver& TR, double& time_s, std::vector<double>& vec)
    {
        double val;
        bool isRA, isAE, isUW;
        time_s = TR.interface->stateRT->getTcpForce(vec);
        printf("[ INFO] tcp_force_est:=< ");
        print_vectord(vec);
        printf(" > [%.3f], ", time_s);
        time_s = TR.interface->stateRT->getTcpForceNorm(val);
        printf("norm:=%.4f  [%.3f]\n", val, time_s);
        TR.interface->stateRT->getKineConfig(isRA, isAE, isUW);
        //snprintf(_msg, 256, "kine_config:=< %d, %d, %d >", (int)isRA, (int)isAE, (int)isUW);
        print_info("kine_config:=< %d, %d, %d >", (int)isRA, (int)isAE, (int)isUW);
        val = TR.interface->stateRT->getSpdDownRatio();
        //snprintf(_msg, 256, "spd_down_ratio:=%.4f", val);
        print_info("spd_down_ratio:=%.4f", val);
        val = TR.interface->stateRT->getSpdJRatio();
        printf("[ INFO] spd_j: ratio:=%.4f, ", val);
        val = TR.interface->stateRT->getSpdJTa();
        printf("Ta:=%.4f\n", val);
        val = TR.interface->stateRT->getSpdLRatio();
        printf("[ INFO] spd_l: spd:=%.4f, ", val);
        val = TR.interface->stateRT->getSpdLTa();
        printf("Ta:=%.4f\n", val);
    }
    void print_rt_8(const TmDriver& TR)
    {
        std::vector<bool> vec;
        TR.interface->stateRT->getDigitalInputMB(vec);
        printf("[ INFO] MB DI: ");
        for (int i = 0; i < vec.size() - 1; i++)
        {
            printf("%d, ", (int)vec[i]);
        }
        printf("%d\n", (int)vec[vec.size() - 1]);
        TR.interface->stateRT->getDigitalOutputMB(vec);
        printf("[ INFO] MB DO: ");
        for (int i = 0; i < vec.size() - 1; i++)
        {
            printf("%d, ", (int)vec[i]);
        }
        printf("%d\n", (int)vec[vec.size() - 1]);
        TR.interface->stateRT->getDigitalInputEE(vec);
        printf("[ INFO] EE DI: ");
        for (int i = 0; i < vec.size() - 1; i++)
        {
            printf("%d, ", (int)vec[i]);
        }
        printf("%d\n", (int)vec[vec.size() - 1]);
        TR.interface->stateRT->getDigitalOutputEE(vec);
        printf("[ INFO] EE DO: ");
        for (int i = 0; i < vec.size() - 1; i++)
        {
            printf("%d, ", (int)vec[i]);
        }
        printf("%d\n", (int)vec[vec.size() - 1]);
    }
    void print_rt_9(const TmDriver& TR, double& time_s)
    {
        bool isErr;
        unsigned char ErrCode;
        print_info("robot_mode:=%d, safety_mode:=%d, teach_mode:=%d, control_mode:=%d",
                   TR.interface->stateRT->getRobotMode(), TR.interface->stateRT->getSafetyMode(),
                   TR.interface->stateRT->getTeachMode(), TR.interface->stateRT->getControlMode()
                  );
        print_info("QueueCmdCount:=%d, BuffEmptyFlag:=%d",
                   TR.interface->stateRT->getQueCmdCount(),
                   TR.interface->stateRT->getBufEmptyFlag()
                  );
        isErr = TR.interface->stateRT->getError(ErrCode, time_s);
        print_info("ErrorCode:=[%d][0x%x] [%.3f]", (int)isErr, ErrCode, time_s);
    }

    std::vector<double> parse_cmd(char* cstr, const char* delim, double& res)
    {
        std::vector<double> ret;
        //int count = 0;
        char* pch;
        char* pch_save;
        pch = strtok_r(cstr, delim, &pch_save);
        //printf("%d: %s\n", count, pch);
        if (pch != NULL)
        {
            while ((pch = strtok_r(NULL, delim, &pch_save)) != NULL)
            {
                //count++;
                if (ret.size() < 6)
                {
                    ret.push_back(atof(pch));
                }
                else
                {
                    res = atof(pch);
                    break;
                }
                //printf("%d: %s\n", count, pch);
            }
        }

        return ret; 
    }
/* tm_moder_driver.cpp */

void ReflexxesStart(TmDriver& TM5)
{
    bool run_succeed = true;
    double SynchronousTime = 3.0;
    std::vector<double> TargetPosition, TargetVelocity;

    RMLPositionInputParameters  *IP_position = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    RMLVelocityInputParameters  *IP_velocity = new RMLVelocityInputParameters(NUMBER_OF_DOFS);

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
    {
        IP_position->CurrentPositionVector->VecData[i] = 0.0;
        IP_position->CurrentVelocityVector->VecData[i] = 0.0;
        IP_position->CurrentAccelerationVector->VecData[i] = 0.0;

        IP_velocity->CurrentPositionVector->VecData[i] = 0.0;
        IP_velocity->CurrentVelocityVector->VecData[i] = 0.0;
        IP_velocity->CurrentAccelerationVector->VecData[i] = 0.0;
    }

    while(run_succeed)
    {
        if (run_succeed)
        {
            TargetPosition = {0,0,0,0,1.57,0};
            TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
            run_succeed = tm_reflexxes::ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        }
        else
            break;

        if (run_succeed)
        {
            TargetPosition = {0,0,0,0,0,0};
            TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
            run_succeed = tm_reflexxes::ReflexxesPositionRun(TM5, *IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        }
        else
            break;
    }

/*    
    while(run_succeed)
    {
        if(run_succeed)
        {
            TargetVelocity = {0,0,0,0,1.0,0};
            run_succeed = ReflexxesVelocityRun(TM5, *IP_velocity, TargetVelocity, SynchronousTime);
        }
        else
            break;
        if (run_succeed)
        {
            TargetVelocity = {0,0,0,0,-1.0,0};
            run_succeed = ReflexxesVelocityRun(TM5, *IP_velocity, TargetVelocity, SynchronousTime);
        }
        else
            break;
    }
*/


    delete IP_position;
    delete IP_velocity;
}


int main(int argc, char **argv)
{

    const int STDIN = 0;
    int sockfd = -1;
    bool fgRun = false;
    std::string host;
    std::condition_variable data_cv;
    std::condition_variable data_cv_rt;


    for (int i = 0; i < argc; i++)
    {
        printf("[DEBUG] arg%d:= %s\n", i, argv[i]);
    }
    host = argv[1];
    printf("[ INFO] host: %s", host.c_str());

    TmDriver TmRobot(data_cv, data_cv_rt, host, 0);

    char cstr[512];
    char delim[] = " ,;\t";
    char c;
    while (1)
    {
/**/
        memset(cstr, 0, 512);
        fgets(cstr, 512, stdin);
        int n = (int)strlen(cstr);
        if (n > 0)
        {
            if (cstr[n - 1] == '\n')
                cstr[n - 1] = '\0';
        }
        if (strncmp(cstr, "quit", 4) == 0)
        {
            TmRobot.interface->halt();
            fgRun = false;
            print_info("quit");
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            break;
        }
        else if (strncmp(cstr, "start", 5) == 0)
        {
            if (!fgRun)
            {
                print_info("start...");
                fgRun = TmRobot.interface->start();
            }
        }
        else if (strncmp(cstr, "halt", 4) == 0)
        {
            print_info("halt");
            TmRobot.interface->halt();
            fgRun = false;
        }
        else if(strncmp(cstr, "home", 4) == 0)
        {
            double blend = 0;
            std::vector<double> vec1 = {0,0,0,0,0,0};
            TmRobot.setMoveJabs(vec1, blend);
            print_info("Back to home");
        }
        else if (strncmp(cstr, "data", 4) == 0)
        {
            double temp_time;
            std::vector<double> temp_vec;
            if (cstr[4] == '\0')
            {
                print_info("data");
            }
            else
            {
                print_info("datart");
                print_info("1:");
                print_rt_1(TmRobot, temp_time, temp_vec);
                print_info("2:");
                print_rt_2(TmRobot, temp_time, temp_vec);
                print_info("3:");
                print_rt_3(TmRobot, temp_time, temp_vec);
                print_info("4:");
                print_rt_4(TmRobot, temp_time, temp_vec);
                print_info("5:");
                print_rt_5(TmRobot, temp_time, temp_vec);
                print_info("6:");
                print_rt_6(TmRobot, temp_time, temp_vec);
                print_info("7:");
                print_rt_7(TmRobot, temp_time, temp_vec);
                print_info("8:");
                print_rt_8(TmRobot);
                print_info("9:");
                print_rt_9(TmRobot, temp_time);
            }
        }
        else if (strncmp(cstr, "clear", 5) == 0)
        {
            system("clear");
        }
/**/        
        else if (strncmp(cstr, "gotest", 6) == 0)
        {
            TmRobot.setJointSpdModeON();
            print_info("joint velocity control mode ON...");
            
            ReflexxesStart(TmRobot);

            TmRobot.setJointSpdModeoOFF();
            print_info("joint vlocity control mode OFF...");
        }
        else if (strncmp(cstr, "movjabs", 7) == 0)
        {
            double blend = 0;
            std::vector<double> vec = parse_cmd(cstr, delim, blend);
            TmRobot.setMoveJabs(vec, blend);
        }
        else
        {
            print_info("send cmd...");
            std::string msg(cstr);
            TmRobot.setCommandMsg(msg);
        }
    }
    return 0;
}




