/*********************************************************************
 *                      Apache License
 *                 Version 2.0, January 2004
 *               http://www.apache.org/licenses/
 *
 * tm_reflexxes.cpp
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

#include "tm_driver/tm_driver.h"

#include "tm_reflexxes/tm_reflexxes.h"

//----------------------------------------------------------------------
//! The functions in this code using Reflexxes API are distributed by 
//!
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

#include "ReflexxesAPI.h"
#include "RMLPositionFlags.h"
#include "RMLPositionInputParameters.h"
#include "RMLPositionOutputParameters.h"

#include "RMLVelocityFlags.h"
#include "RMLVelocityInputParameters.h"
#include "RMLVelocityOutputParameters.h"


namespace tm_reflexxes{

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


    //  ********************************************************************/
    //  fn        : ReflexxesSmoothStop()
    //  beirf     : Use RML velocity based API to stop robot smoothly.
    //  param[in] : TR, Object of TM driver.
    //  param[in] : &InputState, Current State of robot under RML velocity based.
    //  param[in] : TargetVelocity, The final velocity of each joint.
    //  param[in] : SynTime, The time for execute the trajectory.
    //  ********************************************************************
    void ReflexxesSmoothStop(       TmDriver& TR,
                                    RMLVelocityInputParameters &InputState, 
                                    double SynTime)
    {
        double blend = 0, time_s;
        std::vector<double> vec;

        ReflexxesAPI *RML = NULL;
        RMLVelocityInputParameters  *IP = NULL;
        RMLVelocityOutputParameters *OP = NULL;
        RMLVelocityFlags Flags;
        int ResultValue = 0;
        bool pass = true;
        struct timeval tm1,tm2, tm3, tm4;

        RML = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
        IP = new RMLVelocityInputParameters(NUMBER_OF_DOFS);
        OP = new RMLVelocityOutputParameters(NUMBER_OF_DOFS);
        *IP = InputState;


        // ********************************************************************/
        // Creating all relevant objects of the Type II Reflexxes Motion Library*/

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            IP->MaxJerkVector->VecData[i] = 100; //RMLTypeII not using, needed for validity
            IP->MaxAccelerationVector->VecData[i] = 0.5*40;
            IP->TargetVelocityVector->VecData[i] = 0.0;
            if(IP->CurrentVelocityVector->VecData[i] != 0.0)    
                IP->SelectionVector->VecData[i] = true;
            else
                IP->SelectionVector->VecData[i] = false;

        }
        IP->MinimumSynchronizationTime = SynTime;

        // ********************************************************************


        if (IP->CheckForValidity())
            printf("Input values are valid!\n");
        else
            printf("Input values are INVALID!\n");

        Flags.SynchronizationBehavior = RMLFlags::ONLY_TIME_SYNCHRONIZATION;

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

            TR.setMoveJointSpeedabs(vec, blend);

            //**********************
            // Print out commands

            time_s = TR.interface->stateRT->getTime();
            printf("[ %lf ] pos:  ",time_s );

            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                printf("%10.4lf ", OP->NewPositionVector->VecData[i]);

            printf(" | spd: ");

            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                printf("%10.4lf ", OP->NewVelocityVector->VecData[i]);
            
            printf("\n");

            //**********************

            *IP->CurrentPositionVector      =   *OP->NewPositionVector      ;
            *IP->CurrentVelocityVector      =   *OP->NewVelocityVector      ;
            *IP->CurrentAccelerationVector  =   *OP->NewAccelerationVector  ;

            gettimeofday(&tm2, NULL);
            long long time_compensation = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);            
            usleep(24940 - time_compensation);  

            // The area execution in 25ms real time sharp
            //********************************************************
        }

        gettimeofday(&tm4, NULL);
        long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);

        std::vector<double> FinalPosition;
        time_s = TR.interface->stateRT->getQAct(FinalPosition);
        printf("=============== Final state of Smooth Stop =========================\n");
        printf("[ %lf ]  ", time_s);

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            printf(" %10.4lf ",FinalPosition[i]);
        printf("\n");
        print_info("Smooth stop finish in %llu us", tt);
        InputState = *IP;

        delete  RML;
        delete  IP;
        delete  OP;
    }

    //  ********************************************************************/
    //  fn        : ReflexxesVelocityRun()
    //  beirf     : Use RML API to execute given velocity.  
    //  param[in] : TR, Object of TM driver.
    //  param[in] : &InputState, Current State of robot under RML.
    //  param[in] : TargetVelocity, The velocity when reach target position.
    //  param[in] : SynTime, The time for execute the trajectory.
    //  ********************************************************************
    bool ReflexxesVelocityRun(      TmDriver& TR,
                                    RMLVelocityInputParameters &InputState, 
                                    std::vector<double> TargetVelocity, 
                                    double SynTime)
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
        IP  = new RMLVelocityInputParameters(NUMBER_OF_DOFS);
        OP  = new RMLVelocityOutputParameters(NUMBER_OF_DOFS);
        *IP = InputState;


        // ********************************************************************/
        // Creating all relevant objects of the Type II Reflexxes Motion Library*/

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            IP->MaxJerkVector->VecData[i] = 100; //RMLTypeII not using, needed for validity
            IP->MaxAccelerationVector->VecData[i] = 0.5*40;
            IP->TargetVelocityVector->VecData[i] = TargetVelocity[i];

            if(IP->CurrentVelocityVector->VecData[i] != TargetVelocity[i])    
                IP->SelectionVector->VecData[i] = true;
            else
                IP->SelectionVector->VecData[i] = false;
        }

        IP->MinimumSynchronizationTime = SynTime;

        // ********************************************************************


        if (IP->CheckForValidity())
            printf("Input values are valid!\n");
        else
            printf("Input values are INVALID!\n");

        Flags.SynchronizationBehavior = RMLFlags::ONLY_TIME_SYNCHRONIZATION;


        struct timeval tm1,tm2, tm3, tm4;

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

            TR.setMoveJointSpeedabs(vec, blend);

            //**********************
            // Print out commands

            time_s = TR.interface->stateRT->getTime();
            printf("[ %lf ] pos:  ",time_s );
            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                printf("%10.4lf ", OP->NewPositionVector->VecData[i]);

            printf(" | spd: ");
            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                printf("%10.4lf ", OP->NewVelocityVector->VecData[i]);

            printf("\n");

            //***********************
            
            if (kbhit())
            {
                char c = getchar();
                if (c == 'q' || c == 'Q')
                {
                    print_info("Smooth Stop Activate...");
                    //std::vector<double>StopVelocity = {0.0 , 0.0, 0.0, 0.0, 0.0, 0.0};
                    //ReflexxesSmoothStop(TR,*IP, StopVelocity, 0.5);
                    ReflexxesSmoothStop(TR,*IP, 0.5);
                    pass = false;
                    break;
                }
            }

            *IP->CurrentPositionVector      =   *OP->NewPositionVector      ;
            *IP->CurrentVelocityVector      =   *OP->NewVelocityVector      ;
            *IP->CurrentAccelerationVector  =   *OP->NewAccelerationVector  ;

            gettimeofday(&tm2, NULL);
            long long time_compensation = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);            
            usleep(24940 - time_compensation);  

            //********************************************************
            // The area execution in 25ms real time sharp
        }

        gettimeofday(&tm4, NULL);
        long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);

        if(pass)
        {
            std::vector<double> FinalPosition;
            time_s = TR.interface->stateRT->getQAct(FinalPosition);
            printf("=============== Final state velocity based =========================\n");
            printf("[ %lf ]  ", time_s);

            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                printf(" %10.4lf ",FinalPosition[i]);
            printf("\n");
            print_info("Finished in %llu us", tt);
        }

        resetTermios();

        InputState = *IP;

        delete  RML;
        delete  IP;
        delete  OP;

        return pass;
    }

    //  ********************************************************************/
    //  fn        : ReflexxesPositionRun()
    //  beirf     : Use RML API to execute given position.  
    //  param[in] : TR, Object of TM driver.
    //  param[in] : &InputState, Current State of robot under RML.
    //  param[in] : TargetVelocity, The final velocity of each joint.
    //  param[in] : SynTime, The time for execute the trajectory.
    //  ********************************************************************
    bool ReflexxesPositionRun(      TmDriver& TR, 
                                    RMLPositionInputParameters &InputState,
                                    std::vector<double> TargetPosition, 
                                    std::vector<double> TargetVelocity,
                                    double SynTime)
    {
        double blend = 0, time_s;
        std::vector<double> vec, CurrentPosition, FinalPosition;
        bool pass = true;
        struct timeval tm1,tm2, tm3, tm4;

        ReflexxesAPI *RML = NULL    ;
        RMLPositionInputParameters  *IP = NULL;
        RMLPositionOutputParameters *OP = NULL;
        RMLPositionFlags Flags;
        int ResultValue = 0;


        initTermios(1);

        RML = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
        IP = new RMLPositionInputParameters(NUMBER_OF_DOFS);
        OP = new RMLPositionOutputParameters(NUMBER_OF_DOFS);
        *IP = InputState;
        time_s = TR.interface->stateRT->getQAct(CurrentPosition);


        //  ********************************************************************/
        //  Assigning all RMLPositionInputParameters : 
        //  Current POS, VEL, ACC : set before call ReflexxesPositionRun
        //  Target POS, VEL       : set before call ReflexxesPositionRun
        //  Max VEL, ACC          : set after call ReflexxesPositionRun
        //  SelectionVector       : set after call ReflexxesPositionRun
        //  ********************************************************************
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            IP->CurrentPositionVector->VecData[i] = CurrentPosition[i];
            IP->MaxVelocityVector->VecData[i] = MAX_VELOCITY; //0.3247
            IP->MaxAccelerationVector->VecData[i] = MAX_ACC;
            IP->MaxJerkVector->VecData[i] = 100; //RMLTypeII not using, needed for validity
            IP->TargetPositionVector->VecData[i] = TargetPosition[i]; 
            IP->TargetVelocityVector->VecData[i] = TargetVelocity[i];
            IP->SelectionVector->VecData[i] = true;
        }
        IP->MinimumSynchronizationTime = SynTime;

        if (IP->CheckForValidity())
            printf("Input values are valid!\n");
        else
        {
            printf("Input values are INVALID!\n");
            pass = false;
        }

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

            TR.setMoveJointSpeedabs(vec, blend);

            //**********************************
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

            //***********************************

            if (kbhit())
            {
                char c = getchar();
                if (c == 'q' || c == 'Q')
                {
                    print_info("Smooth Stop Activate...");
                    RMLVelocityInputParameters *IP_vel = new RMLVelocityInputParameters(NUMBER_OF_DOFS);
                    *IP_vel->CurrentPositionVector     = *IP->CurrentPositionVector;
                    *IP_vel->CurrentVelocityVector     = *IP->CurrentVelocityVector;
                    *IP_vel->CurrentAccelerationVector = *IP->CurrentAccelerationVector;
                    
                    ReflexxesSmoothStop(TR,*IP_vel, 0.5);
                    delete IP_vel;
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


        if(pass) // print out final state
        {
            time_s = TR.interface->stateRT->getQAct(FinalPosition);
            printf("=============== Final state position based =========================\n");
            printf("[ %lf ]  ", time_s);

            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                printf(" %10.4lf ",FinalPosition[i]);

            printf("\n");
            print_info("Finished in %llu us", tt);
        }

        resetTermios();

        InputState = *IP;

        delete  RML;
        delete  IP;
        delete  OP;

        return pass;
    }

        //  ********************************************************************/
    //  fn        : ReflexxesSmoothStop_sim()
    //  beirf     : Use RML velocity based API to stop robot smoothly.
    //  param[in] : TR, Object of TM driver.
    //  param[in] : &InputState, Current State of robot under RML velocity based.
    //  param[in] : SynTime, The time for execute the trajectory.
    //  ********************************************************************
    void ReflexxesSmoothStop_sim(   RMLVelocityInputParameters &InputState,
                                    double SynTime)
    {
        double time_s;

        ReflexxesAPI *RML = NULL;
        RMLVelocityInputParameters  *IP = NULL;
        RMLVelocityOutputParameters *OP = NULL;
        RMLVelocityFlags Flags;
        
        int ResultValue = 0;
        bool pass = true;
        struct timeval tm1,tm2, tm3, tm4;
        double cycle_iteration = 1.0;

        RML = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);
        IP = new RMLVelocityInputParameters(NUMBER_OF_DOFS);
        OP = new RMLVelocityOutputParameters(NUMBER_OF_DOFS);
        *IP = InputState;

        // ********************************************************************/
        // Creating all relevant objects of the Type II Reflexxes Motion Library*/

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            IP->MaxJerkVector->VecData[i] = 100; //RMLTypeII not using, needed for validity
            IP->MaxAccelerationVector->VecData[i] = 0.5*40;
            IP->TargetVelocityVector->VecData[i] = 0.0;
            
            if(IP->CurrentVelocityVector->VecData[i] != 0.0)    
                IP->SelectionVector->VecData[i] = true;
            else
                IP->SelectionVector->VecData[i] = false;

        }
        IP->MinimumSynchronizationTime = SynTime;

        // ********************************************************************


        if (IP->CheckForValidity())
            printf("Input values are valid!\n");
        else
        {
            printf("Input values are INVALID!\n");
        }

        Flags.SynchronizationBehavior = RMLFlags::ONLY_TIME_SYNCHRONIZATION;

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

            //**********************
            // Print out commands

            time_s = cycle_iteration*0.025;
            cycle_iteration++;
            printf("[ %lf ] pos:  ",time_s );

            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                printf("%10.4lf ", OP->NewPositionVector->VecData[i]);

            printf(" | spd: ");

            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                printf("%10.4lf ", OP->NewVelocityVector->VecData[i]);
            
            printf("\n");

            //**********************

            *IP->CurrentPositionVector      =   *OP->NewPositionVector      ;
            *IP->CurrentVelocityVector      =   *OP->NewVelocityVector      ;
            *IP->CurrentAccelerationVector  =   *OP->NewAccelerationVector  ;

            gettimeofday(&tm2, NULL);
            long long time_compensation = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);            
            usleep(24940 - time_compensation);  

            // The area execution in 25ms real time sharp
            //********************************************************
        }

        gettimeofday(&tm4, NULL);
        long long tt = 1000000 * (tm4.tv_sec - tm3.tv_sec) + (tm4.tv_usec - tm3.tv_usec);

        
        printf("=============== Final state of Smooth Stop =========================\n");
        printf("[ %lf ]  ", time_s);

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
            printf(" %10.4lf ",IP->CurrentPositionVector->VecData[i]);
        printf("\n");
        print_info("Smooth stop finish in %llu us", tt);
        InputState = *IP;

        delete  RML;
        delete  IP;
        delete  OP;
    }

    //  ********************************************************************/
    //  fn        : ReflexxesVelocityRun_sim()
    //  beirf     : Use RML API to execute given velocity in simulation.  
    //  param[in] : &InputState, Current State of robot under RML.
    //  param[in] : TargetVelocity, The velocity when reach target position.
    //  param[in] : SynTime, The time for execute the trajectory.
    //  ********************************************************************
    bool ReflexxesVelocityRun_sim(  RMLVelocityInputParameters &InputState, 
                                    std::vector<double> TargetVelocity, 
                                    double SynTime)
    {
        double time_s;

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
        // Creating all relevant objects of the Type II Reflexxes Motion Library
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            IP->MaxJerkVector->VecData[i] = 100; //RMLTypeII not using, needed for validity
            IP->MaxAccelerationVector->VecData[i] = 0.5*40;
            IP->TargetVelocityVector->VecData[i] = TargetVelocity[i];

            if(IP->CurrentVelocityVector->VecData[i] != TargetVelocity[i])    
                IP->SelectionVector->VecData[i] = true;
            else
                IP->SelectionVector->VecData[i] = false;
        }
        IP->MinimumSynchronizationTime = SynTime;

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
        double cycle_iteration = 1.0;

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

            //***************************************************************
            // Print out commands

            time_s = cycle_iteration*0.025;
            cycle_iteration++;

            printf("[ %lf ] pos:  ",time_s);

            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                printf("%10.4lf ", OP->NewPositionVector->VecData[i]);

            printf(" | spd: ");

            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                printf("%10.4lf ", OP->NewVelocityVector->VecData[i]);
    
            printf("\n");

            //***************************************************************
            if (kbhit())
            {
                char c = getchar();
                if (c == 'q' || c == 'Q')
                {
                    print_info("Smooth Stop Activate...");
                    ReflexxesSmoothStop_sim(*IP, 0.5);
                    pass = false;
                    break;
                }
            }

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

        if(pass)
        {
            printf("=============== Final state velocity based =========================\n");
            printf("[ %lf ]  ", time_s);

            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                printf(" %10.4lf ",IP->CurrentVelocityVector->VecData[i]);
            printf("\n");
            print_info("Finished in %llu us", tt);
        }

        resetTermios();

        InputState = *IP;
        delete  RML;
        delete  IP;
        delete  OP;

        return pass;
    }

    //  ********************************************************************/
    //  fn        : ReflexxesVelocityRun_sim() [Overloaded]
    //  beirf     : Use RML API to execute given velocity in simulation.  
    //  param[in] : &InputState, Current State of robot under RML.
    //  param[in] : TargetVelocity, The velocity when reach target position.
    //  ********************************************************************
    bool ReflexxesVelocityRun_sim(  RMLVelocityInputParameters &InputState, 
                                    std::vector<double> TargetVelocity)
    {
        double time_s;

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
        // Creating all relevant objects of the Type II Reflexxes Motion Library
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            IP->MaxJerkVector->VecData[i] = 100; //RMLTypeII not using, needed for validity
            IP->MaxAccelerationVector->VecData[i] = 0.5*40;
            IP->TargetVelocityVector->VecData[i] = TargetVelocity[i];

            if(IP->CurrentVelocityVector->VecData[i] != TargetVelocity[i])    
                IP->SelectionVector->VecData[i] = true;
            else
                IP->SelectionVector->VecData[i] = false;
        }

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
        double cycle_iteration = 1.0;

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

            //***************************************************************
            // Print out commands

            time_s = cycle_iteration*0.025;
            cycle_iteration++;

            printf("[ %lf ] pos:  ",time_s);

            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                printf("%10.4lf ", OP->NewPositionVector->VecData[i]);

            printf(" | spd: ");

            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                printf("%10.4lf ", OP->NewVelocityVector->VecData[i]);
    
            printf("\n");

            //***************************************************************
            if (kbhit())
            {
                char c = getchar();
                if (c == 'q' || c == 'Q')
                {
                    print_info("Smooth Stop Activate...");
                    ReflexxesSmoothStop_sim(*IP, 0.5);
                    pass = false;
                    break;
                }
            }

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

        if(pass)
        {
            printf("=============== Final state velocity based =========================\n");
            printf("[ %lf ]  ", time_s);

            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                printf(" %10.4lf ",IP->CurrentVelocityVector->VecData[i]);
            printf("\n");
            print_info("Finished in %llu us", tt);
        }

        resetTermios();

        InputState = *IP;
        delete  RML;
        delete  IP;
        delete  OP;

        return pass;
    }

    //  ********************************************************************/
    //  fn        : ReflexxesPositionRun_sim()
    //  beirf     : Use RML API to execute given position in silumation.  
    //  param[in] : &InputState, Current State of robot under RML.
    //  param[in] : TargetVelocity, The final velocity of each joint.
    //  param[in] : SynTime, The time for execute the trajectory.
    //  ********************************************************************
    bool ReflexxesPositionRun_sim(  RMLPositionInputParameters &InputState, 
                                    std::vector<double> TargetPosition,
                                    std::vector<double> TargetVelocity, 
                                    double SynTime)
    {
        double time_s;
        std::vector<double> FinalPosition;
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

        *IP = InputState;

        //  ********************************************************************/
        //  Assigning all RMLPositionInputParameters : 
        //  Current POS, VEL, ACC : set before call ReflexxesPositionRun
        //  Target POS, VEL       : set before call ReflexxesPositionRun
        //  Max VEL, ACC          : set after call ReflexxesPositionRun
        //  SelectionVector       : set after call ReflexxesPositionRun
        //  ********************************************************************
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            IP->MaxJerkVector->VecData[i] = 100; //RMLTypeII not using, needed for validity
            IP->MaxVelocityVector->VecData[i] = MAX_VELOCITY; //0.3247
            IP->MaxAccelerationVector->VecData[i] = MAX_ACC;
            IP->TargetPositionVector->VecData[i] = TargetPosition[i]; 
            IP->TargetVelocityVector->VecData[i] = TargetVelocity[i];
            IP->SelectionVector->VecData[i] = true;
        }
        IP->MinimumSynchronizationTime = SynTime;


        if (IP->CheckForValidity())
            printf("Input values are valid!\n");
        else
        {
            printf("Input values are INVALID!\n");
            pass = false;
        }


        struct timeval tm1,tm2, tm3, tm4;
        double cycle_iteration = 1.0;

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
                    print_info("Smooth Stop Activate...");
                    RMLVelocityInputParameters *IP_vel = new RMLVelocityInputParameters(NUMBER_OF_DOFS);

                    *IP_vel->CurrentPositionVector     = *IP->CurrentPositionVector;
                    *IP_vel->CurrentVelocityVector     = *IP->CurrentVelocityVector;
                    *IP_vel->CurrentAccelerationVector = *IP->CurrentAccelerationVector;
                    ReflexxesSmoothStop_sim(*IP_vel, 0.5);
                    delete IP_vel;
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

        if(pass)
        {
            printf("=============== Final state =========================\n");

            for (int i = 0; i < NUMBER_OF_DOFS; ++i)
                printf("%10.4lf ", IP->CurrentPositionVector->VecData[i]);

            printf("\n");
            print_info("Finished in %llu us", tt);
        }
        resetTermios();
        InputState = *IP;

        delete  RML;
        delete  IP;
        delete  OP;

        return pass;
    }
}







