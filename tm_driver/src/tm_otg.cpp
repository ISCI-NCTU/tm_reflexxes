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


#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>


//*************************************************************************
// defines

#define CYCLE_TIME_IN_SECONDS                   0.025
#define NUMBER_OF_DOFS                          6
#define DEG2RAD 0.01745329252
#define RAD2DEG 57.29577951

#define MAX_VELOCITY 1.0
#define MAX_ACC 1.5

double cubic_polynomial(double time_stamp)
{
    double a = -0.1163;
    double b = 0.523;
    double qd;

    qd = 3*a*time_stamp*time_stamp + 2*b*time_stamp;

    return qd;
}

int main()
{
    // ********************************************************************
    // Variable declarations and definitions

    bool                        FirstCycleCompleted         =   false   ;

    int                         ResultValue                 =   0
                            ,   i                           =   0
                            ,   j                           =   0       ;

    ReflexxesAPI                *RML                        =   NULL    ;

    RMLPositionInputParameters  *IP                         =   NULL    ;

    RMLPositionOutputParameters *OP                         =   NULL    ;

    RMLPositionFlags            Flags                                   ;

    // ********************************************************************
    // Creating all relevant objects of the Type II Reflexxes Motion Library

    RML = new ReflexxesAPI(NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS);

    IP = new RMLPositionInputParameters(NUMBER_OF_DOFS);

    OP = new RMLPositionOutputParameters(NUMBER_OF_DOFS);


    IP->CurrentPositionVector->VecData[0] = 0.0;
    IP->CurrentPositionVector->VecData[1] = 0.0;
    IP->CurrentPositionVector->VecData[2] = 0.0;
    IP->CurrentPositionVector->VecData[3] = 0.0;
    IP->CurrentPositionVector->VecData[4] = 0.0;
    IP->CurrentPositionVector->VecData[5] = 0.0;

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

    IP->TargetPositionVector->VecData[0] = 0.0;
    IP->TargetPositionVector->VecData[1] = 0.0;
    IP->TargetPositionVector->VecData[2] = 1.57;
    IP->TargetPositionVector->VecData[3] = -1.57;
    IP->TargetPositionVector->VecData[4] = 1.57;
    IP->TargetPositionVector->VecData[5] = 0.0;

    IP->TargetVelocityVector->VecData[0] = 0.0;
    IP->TargetVelocityVector->VecData[1] = 0.0;
    IP->TargetVelocityVector->VecData[2] = 0.0;
    IP->TargetVelocityVector->VecData[3] = 0.0;
    IP->TargetVelocityVector->VecData[4] = 0.0;
    IP->TargetVelocityVector->VecData[5] = 0.0;

    IP->SelectionVector->VecData[0] = false;
    IP->SelectionVector->VecData[1] = false;
    IP->SelectionVector->VecData[2] = false;
    IP->SelectionVector->VecData[3] = false;
    IP->SelectionVector->VecData[4] = true;
    IP->SelectionVector->VecData[5] = false;

    // ********************************************************************
    // Specifying the minimum synchronization time

    IP->MinimumSynchronizationTime              =   2.0         ;

    // ********************************************************************
    // Checking the input parameters (optional)

    if (IP->CheckForValidity())
        printf("Input values are valid!\n");
    else
        printf("Input values are INVALID!\n");

    // ********************************************************************
    // Starting the control loop



    float cycle_iteration = 0;
    struct timeval tm1, tm3, tm4;
    struct timeval tm2, tm5;

	gettimeofday(&tm5, NULL);

    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {

        //gettimeofday(&tm1, NULL);
	
        ResultValue =  RML->RMLPosition(*IP, OP, Flags );

        if (ResultValue < 0)
        {
            printf("An error occurred (%d).\n", ResultValue );
            break;
        }

        double timenow = cycle_iteration++*0.025;
        //ouble jpd = cubic_polynomial(timenow);

        printf(" [%.3f] ",timenow);
        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            printf("%10.4lf ", OP->NewPositionVector->VecData[i]);
        }

        printf(" | ");

        for (int i = 0; i < NUMBER_OF_DOFS; ++i)
        {
            printf("%10.4lf ", OP->NewVelocityVector->VecData[i]);
        }
        printf("\n");


        *IP->CurrentPositionVector =  *OP->NewPositionVector;
        *IP->CurrentVelocityVector =  *OP->NewVelocityVector;
        //usleep(25000);
        /*
        gettimeofday(&tm2, NULL);
        long long t = 1000000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec);

        usleep(24900-t);

	    gettimeofday(&tm4, NULL);
   		long long tt = 1000000 * (tm4.tv_sec - tm1.tv_sec) + (tm4.tv_usec - tm1.tv_usec);
		printf("  cycle time = %llu us \n",tt); */
    }

	gettimeofday(&tm3, NULL);
	long long ttt = 1000000 * (tm3.tv_sec - tm5.tv_sec) + (tm3.tv_usec - tm5.tv_usec);
	printf("all time = %llu us \n",ttt); 


    delete  RML;
    delete  IP;
    delete  OP;
    exit(EXIT_SUCCESS);
}
