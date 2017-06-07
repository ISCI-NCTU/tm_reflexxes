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
/**************************** Original source code *******************
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

** Modullize Reflexxes_position_run()
1. Position input use joint position.
2. Position use cartesian position(import T).

** Create Reflexxes_velocity_run()
1. Velocity input use joint space.
2. Velocity use cartesian space(import jacobian)

*********************************/

#include "src/tm_driver/tm_driver.h"

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#include <termios.h>
#include <math.h>
#include <iostream>
#include <sys/time.h>
#include <unistd.h>

#define DEG2RAD 0.01745329252
#define RAD2DEG 57.29577951

#define CYCLE_TIME_IN_SECONDS 0.025
#define NUMBER_OF_DOFS 6

#define MAX_VELOCITY 1.0
#define MAX_ACC 1.5

using namespace std;

static struct termios oldt, newt;
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

bool Reflexxes_position_run(TmDriver& TR, std::vector<double> TargetPosition, double synTime)
{
    double blend = 0, time_s;
    std::vector<double> vec, CurrentPosition, FinalPosition;
    bool pass = true;

    ReflexxesAPI *RML = NULL    ;
    RMLPositionInputParameters  *IP = NULL;
    RMLPositionOutputParameters *OP = NULL;
    RMLPositionFlags Flags;
    int ResultValue = 0;


    initTermios(1);

    RML = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
    IP = new RMLPositionInputParameters(NUMBER_OF_DOFS);
    OP = new RMLPositionOutputParameters(NUMBER_OF_DOFS);

    time_s = TR.interface->stateRT->getQAct(CurrentPosition);


    // ********************************************************************/
    // Creating all relevant objects of the Type II Reflexxes Motion Library*/

    IP->CurrentPositionVector->VecData[0] = CurrentPosition[0];
    IP->CurrentPositionVector->VecData[1] = CurrentPosition[1];
    IP->CurrentPositionVector->VecData[2] = CurrentPosition[2];
    IP->CurrentPositionVector->VecData[3] = CurrentPosition[3];
    IP->CurrentPositionVector->VecData[4] = CurrentPosition[4];
    IP->CurrentPositionVector->VecData[5] = CurrentPosition[5];

    IP->CurrentVelocityVector->VecData[0] = 0.0;
    IP->CurrentVelocityVector->VecData[1] = 0.0;
    IP->CurrentVelocityVector->VecData[2] = 0.0;
    IP->CurrentVelocityVector->VecData[3] = 0.0;
    IP->CurrentVelocityVector->VecData[4] = 0.0;
    IP->CurrentVelocityVector->VecData[5] = 0.0;

    IP->CurrentAccelerationVector->VecData[0] = 0.0;
    IP->CurrentAccelerationVector->VecData[1] = 0.0;
    IP->CurrentAccelerationVector->VecData[2] = 0.0;
    IP->CurrentAccelerationVector->VecData[3] = 0.0;
    IP->CurrentAccelerationVector->VecData[4] = 0.0;
    IP->CurrentAccelerationVector->VecData[5] = 0.0;

    IP->MaxVelocityVector->VecData[0] = MAX_VELOCITY; //0.3247
    IP->MaxVelocityVector->VecData[1] = MAX_VELOCITY; //0.3247
    IP->MaxVelocityVector->VecData[2] = MAX_VELOCITY; //0.3247
    IP->MaxVelocityVector->VecData[3] = MAX_VELOCITY; //0.3247
    IP->MaxVelocityVector->VecData[4] = MAX_VELOCITY; //0.3247
    IP->MaxVelocityVector->VecData[5] = MAX_VELOCITY; //0.3247

    IP->MaxAccelerationVector->VecData[0] = MAX_ACC;
    IP->MaxAccelerationVector->VecData[1] = MAX_ACC;
    IP->MaxAccelerationVector->VecData[2] = MAX_ACC;
    IP->MaxAccelerationVector->VecData[3] = MAX_ACC;
    IP->MaxAccelerationVector->VecData[4] = MAX_ACC;
    IP->MaxAccelerationVector->VecData[5] = MAX_ACC;

    IP->TargetPositionVector->VecData[0] = TargetPosition[0];
    IP->TargetPositionVector->VecData[1] = TargetPosition[1];
    IP->TargetPositionVector->VecData[2] = TargetPosition[2];
    IP->TargetPositionVector->VecData[3] = TargetPosition[3];
    IP->TargetPositionVector->VecData[4] = TargetPosition[4];
    IP->TargetPositionVector->VecData[5] = TargetPosition[5];

    IP->TargetVelocityVector->VecData[0] = 0.0;
    IP->TargetVelocityVector->VecData[1] = 0.0;
    IP->TargetVelocityVector->VecData[2] = 0.0;
    IP->TargetVelocityVector->VecData[3] = 0.0;
    IP->TargetVelocityVector->VecData[4] = 0.0;
    IP->TargetVelocityVector->VecData[5] = 0.0;

    IP->SelectionVector->VecData[0] = true;
    IP->SelectionVector->VecData[1] = true;
    IP->SelectionVector->VecData[2] = true;
    IP->SelectionVector->VecData[3] = true;
    IP->SelectionVector->VecData[4] = true;
    IP->SelectionVector->VecData[5] = false;

    IP->MinimumSynchronizationTime = synTime;

    // ********************************************************************


    if (IP->CheckForValidity())
        printf("Input values are valid!\n");
    else
    {
        printf("Input values are INVALID!\n");
        pass = false;
    }


    struct timeval tm1,tm2, tm3, tm4;

    gettimeofday(&tm3, NULL);

    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
        //********************************************************
        // The area execution in 25ms real time sharp

        gettimeofday(&tm1, NULL); 
        if (kbhit())
        {
            char c = getchar();
            if (c == 'q' || c == 'Q')
            {
                print_info("stop...");
                TR.setRobotStop();
                pass = false;
                break;
            }
        }

        ResultValue =  RML->RMLPosition(*IP, OP, Flags );

        if (ResultValue < 0)
        {
            printf("An error occurred (%d).\n", ResultValue );
            break;
        }
        vec = { OP->NewVelocityVector->VecData[0],
                OP->NewVelocityVector->VecData[1],
                OP->NewVelocityVector->VecData[2],
                OP->NewVelocityVector->VecData[3],
                OP->NewVelocityVector->VecData[4],
                OP->NewVelocityVector->VecData[5]};

        TR.setMoveJointSpeedabs(vec, blend);

        //***************************************************************
        // Print out commands

        time_s = TR.interface->stateRT->getTime();
        printf("[ %lf ] pos:  ",time_s );

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            printf("%10.4lf ", OP->NewPositionVector->VecData[i]);
        }

        printf(" | spd: ");

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            printf("%10.4lf ", OP->NewVelocityVector->VecData[i]);
        }
        printf("\n");

        //***************************************************************


        *IP->CurrentPositionVector =  *OP->NewPositionVector;
        *IP->CurrentVelocityVector =  *OP->NewVelocityVector;

        gettimeofday(&tm2, NULL);
        long long time_compensation = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);            
        usleep(24940 - time_compensation);  

        //********************************************************
        // The area execution in 25ms real time sharp
    }

    gettimeofday(&tm4, NULL);
    long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);



    time_s = TR.interface->stateRT->getQAct(FinalPosition);
    printf("=============== Final state =========================\n");
    printf("[ %lf ]  ", time_s);

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        printf(" %10.4lf ",FinalPosition[i]);

    printf("\n");
    print_info("Finished in %llu us", tt);

    resetTermios();

    delete  RML;
    delete  IP;
    delete  OP;

    return pass;
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
        else if (strncmp(cstr, "gotest", 6) == 0)
        {
            TmRobot.setJointSpdModeON();
            print_info("joint velocity control mode ON...");
            bool run_succeed = true;
            double SynchronousTime = 6.0;
            std::vector<double> TargetPosition;

            while(run_succeed)
            {
                if (run_succeed)
                {
                    TargetPosition = {0,0,0,0,1.57,0};
                    run_succeed = Reflexxes_position_run(TmRobot, TargetPosition, SynchronousTime);
                }
                else
                    break;

                if (run_succeed)
                {
                    TargetPosition = {0,0,0,0,0,0};
                    run_succeed = Reflexxes_position_run(TmRobot, TargetPosition, SynchronousTime);
                }
                else
                    break;
            }

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
