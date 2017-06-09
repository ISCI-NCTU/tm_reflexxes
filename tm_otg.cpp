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

//---------------------- Original source ----------------------------------
//! \copyright Copyright (C) 2014 Google, Inc.
//! \n
//! \n
//! <b>GNU Lesser General Public License</b>
//! \n
//! \n
//! This file is part of the Type II Reflexxes Motion Library.
//! \n\n
//! The Type II Reflexxes Motion Library is free software: you can redistribute
//! it and/or modify it under the terms of the GNU Lesser General Public License
//! as published by the Free Software Foundation, either version 3 of the
//! License, or (at your option) any later version.
//! \n\n
//! The Type II Reflexxes Motion Library is distributed in the hope that it
//! will be useful, but WITHOUT ANY WARRANTY; without even the implied
//! warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
//! the GNU Lesser General Public License for more details.
//! \n\n
//! You should have received a copy of the GNU Lesser General Public License
//! along with the Type II Reflexxes Motion Library. If not, see
//! <http://www.gnu.org/licenses/>.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


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

#include "src/tm_driver/tm_driver.h"

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#include <RMLVelocityFlags.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>

//*************************************************************************
// defines

#define CYCLE_TIME_IN_SECONDS                   0.025
#define NUMBER_OF_DOFS                          6
#define DEG2RAD 0.01745329252
#define RAD2DEG 57.29577951

#define MAX_VELOCITY 1.0
#define MAX_ACC 0.0375*40


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

double cubic_polynomial(double time_stamp)
{
    double a = -0.1163;
    double b = 0.523;
    double qd;

    qd = 3*a*time_stamp*time_stamp + 2*b*time_stamp;

    return qd;
}

bool VelocityState(RMLVelocityInputParameters &InputState, int joint_num)
{
    RMLVelocityInputParameters  *IP = NULL;
    IP = new RMLVelocityInputParameters(NUMBER_OF_DOFS);
    *IP = InputState;
    if(IP->CurrentVelocityVector->VecData[joint_num] != 0.0)
        return true;
    else
        return false;
}

/**********************************************************
* Intro: This function generate a smooth stop traj.
* Input : InputState : 
{
    CurrentPosition
    CurrentVelocity
}
**********************************************************/


bool Reflexxes_velocity_run(    RMLVelocityInputParameters &InputState, 
                                std::vector<double> TargetVelocity, 
                                double synTime,
                                bool already_selected )
{
    double blend = 0, time_s;
    std::vector<double> vec;

    ReflexxesAPI *RML = NULL;
    RMLVelocityInputParameters  *IP = NULL;
    RMLVelocityOutputParameters *OP = NULL;
    RMLVelocityFlags Flags;
    int ResultValue = 0;
    bool pass = true;

    initTermios(1);

    RML = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
    IP = new RMLVelocityInputParameters(NUMBER_OF_DOFS);
    OP = new RMLVelocityOutputParameters(NUMBER_OF_DOFS);
    *IP = InputState;


    // ********************************************************************/
    // Creating all relevant objects of the Type II Reflexxes Motion Library*/
/**/
/*
    IP->CurrentPositionVector->VecData[0] = 0.0;
    IP->CurrentPositionVector->VecData[1] = 0.0;
    IP->CurrentPositionVector->VecData[2] = 1.57;
    IP->CurrentPositionVector->VecData[3] = -1.57;
    IP->CurrentPositionVector->VecData[4] = 1.57;
    IP->CurrentPositionVector->VecData[5] = 0.0;

    IP->CurrentVelocityVector->VecData[0] = 0.0;
    IP->CurrentVelocityVector->VecData[1] = 0.0;
    IP->CurrentVelocityVector->VecData[2] = 1.0;
    IP->CurrentVelocityVector->VecData[3] = 1.0;
    IP->CurrentVelocityVector->VecData[4] = 1.0;
    IP->CurrentVelocityVector->VecData[5] = 0.0;

    IP->CurrentAccelerationVector->VecData[0] = 0.0;
    IP->CurrentAccelerationVector->VecData[1] = 0.0;
    IP->CurrentAccelerationVector->VecData[2] = 0.0;
    IP->CurrentAccelerationVector->VecData[3] = 0.0;
    IP->CurrentAccelerationVector->VecData[4] = 0.0;
    IP->CurrentAccelerationVector->VecData[5] = 0.0;
*/

    IP->MaxAccelerationVector->VecData[0] = 0.5*40;
    IP->MaxAccelerationVector->VecData[1] = 0.5*40;
    IP->MaxAccelerationVector->VecData[2] = 0.5*40;
    IP->MaxAccelerationVector->VecData[3] = 0.5*40;
    IP->MaxAccelerationVector->VecData[4] = 0.5*40;
    IP->MaxAccelerationVector->VecData[5] = 0.5*40;

    IP->TargetVelocityVector->VecData[0] = TargetVelocity[0];
    IP->TargetVelocityVector->VecData[1] = TargetVelocity[1];
    IP->TargetVelocityVector->VecData[2] = TargetVelocity[2];
    IP->TargetVelocityVector->VecData[3] = TargetVelocity[3];
    IP->TargetVelocityVector->VecData[4] = TargetVelocity[4];
    IP->TargetVelocityVector->VecData[5] = TargetVelocity[5];

    if(!already_selected)
    {
        IP->SelectionVector->VecData[0] = true;
        IP->SelectionVector->VecData[1] = true;
        IP->SelectionVector->VecData[2] = true;
        IP->SelectionVector->VecData[3] = true;
        IP->SelectionVector->VecData[4] = true;
        IP->SelectionVector->VecData[5] = false;
    }

    IP->MinimumSynchronizationTime = synTime;

    // ********************************************************************


    if (IP->CheckForValidity())
    {
        printf("Input values are valid!\n");
    }
    else
    {
        printf("Input values are INVALID!\n");
    }
    Flags.SynchronizationBehavior = RMLFlags::ONLY_TIME_SYNCHRONIZATION;
    


    struct timeval tm1,tm2, tm3, tm4;
    double cycle_iteration = 0.0;

    gettimeofday(&tm3, NULL);

    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
        //********************************************************
        // The area execution in 25ms real time sharp

        gettimeofday(&tm1, NULL); 

        ResultValue =  RML->RMLVelocity(*IP, OP, Flags );

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

        //***************************************************************
        // Print out commands

        time_s = cycle_iteration*0.025;
        cycle_iteration++;

        printf("[ %lf ] pos:  ",time_s);

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
        if (kbhit())
        {
            char c = getchar();
            if (c == 'q' || c == 'Q')
            {
                print_info("stop...");
                std::vector<double>TargetVelocity = {0.0 , 0.0, 0.0, 0.0, 0.0, 0.0};

                for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                    IP->SelectionVector->VecData[i] = VelocityState(*IP, i);

                Reflexxes_velocity_run(*IP, TargetVelocity, 0.5, true);
                pass = false;
                break;
            }
        }

        if(time_s > 5.0)
            break;

        *IP->CurrentPositionVector =  *OP->NewPositionVector;
        *IP->CurrentVelocityVector =  *OP->NewVelocityVector;
        *IP->CurrentAccelerationVector = *OP->NewAccelerationVector;

        gettimeofday(&tm2, NULL);
        long long time_compensation = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);            
        usleep(24940 - time_compensation);  

        //********************************************************
        // The area execution in 25ms real time sharp
    }

    gettimeofday(&tm4, NULL);
    long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);

    delete  RML;
    delete  IP;
    delete  OP;

    return pass;
}

bool Reflexxes_position_run(std::vector<double> CurrentPosition, std::vector<double> TargetPosition, double synTime)
{
    double blend = 0, time_s;
    std::vector<double> vec, FinalPosition;
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


    // ********************************************************************/
    // Creating all relevant objects of the Type II Reflexxes Motion Library*/
/**/
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
    IP->SelectionVector->VecData[5] = true;

    IP->MinimumSynchronizationTime = synTime;
/**/
    // ********************************************************************


    if (IP->CheckForValidity())
        printf("Input values are valid!\n");
    else
    {
        printf("Input values are INVALID!\n");
        pass = false;
    }


    struct timeval tm1,tm2, tm3, tm4;
    double cycle_iteration = 0.0;

    gettimeofday(&tm3, NULL);

    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
        //********************************************************
        // The area execution in 25ms real time sharp

        gettimeofday(&tm1, NULL); 

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

        //***************************************************************
        // Print out commands

        time_s = cycle_iteration*0.025;
        cycle_iteration++;

        printf("[ %lf ] pos:  ",time_s);

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

        if (kbhit())
        {
            char c = getchar();
            if (c == 'q' || c == 'Q')
            {
                print_info("stop...");
                
                RMLVelocityInputParameters *IP_vel =  new RMLVelocityInputParameters(NUMBER_OF_DOFS);
                *IP_vel->CurrentPositionVector = *IP->CurrentPositionVector;
                *IP_vel->CurrentVelocityVector = *IP->CurrentVelocityVector;
                *IP_vel->CurrentAccelerationVector = *IP->CurrentAccelerationVector;

                for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                {
                    if(IP->CurrentVelocityVector->VecData[i] == 0.0)
                        IP_vel->SelectionVector->VecData[i] = false;
                    else
                        IP_vel->SelectionVector->VecData[i] = true;
                }
                
                std::vector<double>TargetVelocity = {0,0,0,0,0,0};

                Reflexxes_velocity_run(*IP_vel, TargetVelocity, 0.5, true);
                pass = false;
                break;
            }
        }

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

    printf("=============== Final state =========================\n");

    for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        printf("%10.4lf ", OP->NewPositionVector->VecData[i]);

    printf("\n");
    print_info("Finished in %llu us", tt);

    resetTermios();

    delete  RML;
    delete  IP;
    delete  OP;

    return pass;
}


int main(int argc, char const *argv[])
{
    print_info("joint velocity control mode ON...");
    bool run_succeed = true;
    double SynchronousTime = 2.0;
    std::vector<double> TargetPosition, CurrentPosition; 


    while(run_succeed)
    {
        if (run_succeed)
        {
            TargetPosition =  {0,0,1.57,-1.57,1.57,0};
            CurrentPosition = {0,0,0,0,0,0};
            run_succeed = Reflexxes_position_run(CurrentPosition, TargetPosition, SynchronousTime);
        }
        else
            break;

        if (run_succeed)
        {
            CurrentPosition = TargetPosition;
            TargetPosition = {0,0,0,0,0,0};
            run_succeed = Reflexxes_position_run(CurrentPosition, TargetPosition, SynchronousTime);
        }
        else
            break;
    }
/*
    RMLVelocityInputParameters *IP_vel =  new RMLVelocityInputParameters(NUMBER_OF_DOFS);
    std::vector<double>TargetVelocity = {0,0,0,0,0,0};

    run_succeed = Reflexxes_velocity_run(*IP_vel, TargetVelocity, 0.5);
*/

    print_info("joint vlocity control mode OFF...");

    return 0;
}
