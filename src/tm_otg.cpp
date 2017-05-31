/*********************************************************************
 *          GNU LESSER GENERAL PUBLIC LICENSE
 *              Version 3, 29 June 2007
 * 
 *Copyright (c) 2017, Howard Chen, ISCI lab, NCTU.
 * 
 *Author :  Howard Chen
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

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>


//*************************************************************************
// defines

#define CYCLE_TIME_IN_SECONDS                   0.025
#define NUMBER_OF_DOFS                          1

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


    IP->CurrentPositionVector->VecData      [0] =    1.0487      ;
    //IP->CurrentPositionVector->VecData      [1] =      0.0      ;
    //IP->CurrentPositionVector->VecData      [2] =     50.0      ;

    IP->CurrentVelocityVector->VecData      [0] =    0.0      ;
    //IP->CurrentVelocityVector->VecData      [1] =   -220.0      ;
    //IP->CurrentVelocityVector->VecData      [2] =    -50.0      ;

    IP->CurrentAccelerationVector->VecData  [0] =   -0.5184      ;
    //IP->CurrentAccelerationVector->VecData  [1] =    250.0      ;
    //IP->CurrentAccelerationVector->VecData  [2] =    -50.0      ;

    IP->MaxVelocityVector->VecData          [0] =    0.317      ;
    //IP->MaxVelocityVector->VecData          [1] =    100.0      ;
    //IP->MaxVelocityVector->VecData          [2] =    300.0      ;

    IP->MaxAccelerationVector->VecData      [0] =     3.18;
    //IP->MaxAccelerationVector->VecData      [1] =    200.0      ;
    //IP->MaxAccelerationVector->VecData      [2] =    100.0      ;

    IP->MaxJerkVector->VecData              [0] =    141.004     ;
    //IP->MaxJerkVector->VecData              [1] =    300.0      ;
    //IP->MaxJerkVector->VecData              [2] =    200.0      ;

    IP->TargetPositionVector->VecData       [0] =   1.838      ;
    //IP->TargetPositionVector->VecData       [1] =   -200.0      ;
    //IP->TargetPositionVector->VecData       [2] =   -350.0      ;

    IP->TargetVelocityVector->VecData       [0] =    0.098       ;
    //IP->TargetVelocityVector->VecData       [1] =   -50.0       ;
    //IP->TargetVelocityVector->VecData       [2] =  -200.0       ;

    IP->SelectionVector->VecData            [0] =   true        ;
    //IP->SelectionVector->VecData            [1] =   true        ;
    //IP->SelectionVector->VecData            [2] =   true        ;

    // ********************************************************************
    // Specifying the minimum synchronization time

    IP->MinimumSynchronizationTime              =   3.0         ;

    // ********************************************************************
    // Checking the input parameters (optional)

    if (IP->CheckForValidity())
    {
        printf("Input values are valid!\n");
    }
    else
    {
        printf("Input values are INVALID!\n");
    }

    // ********************************************************************
    // Starting the control loop

    while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {

        // ****************************************************************
        // Wait for the next timer tick
        // (not implemented in this example in order to keep it simple)
        // ****************************************************************

        // Calling the Reflexxes OTG algorithm
        ResultValue =  RML->RMLPosition(*IP, OP, Flags );

        if (ResultValue < 0)
        {
            printf("An error occurred (%d).\n", ResultValue );
            break;
        }

        // ****************************************************************
        // The following part completely describes all output values
        // of the Reflexxes Type II Online Trajectory Generation
        // algorithm.

        if (!FirstCycleCompleted)
        {
            //FirstCycleCompleted =   true;

            /*printf("-------------------------------------------------------\n");
            printf("General information:\n\n");

            printf("The execution time of the current trajectory is %.3lf seconds.\n", OP->GetSynchronizationTime());

            if (OP->IsTrajectoryPhaseSynchronized())
            {
                printf("The current trajectory is phase-synchronized.\n");
            }
            else
            {
                printf("The current trajectory is time-synchronized.\n");
            }
            if (OP->WasACompleteComputationPerformedDuringTheLastCycle())
            {
                printf("The trajectory was computed during the last computation cycle.\n");
            }
            else
            {
                printf("The input values did not change, and a new computation of the trajectory parameters was not required.\n");
            }

            printf("-------------------------------------------------------\n");
            printf("New state of motion:\n\n");
*/

            printf("%10.3lf ", OP->NewPositionVector->VecData[j]);
            printf("%10.3lf ", OP->NewVelocityVector->VecData[j]);
            printf("%10.3lf ", OP->NewAccelerationVector->VecData[j]);
            printf("\n");
/*
            printf("New position/pose vector                  : ");
            for ( j = 0; j < NUMBER_OF_DOFS; j++)
            {
                printf("%10.3lf ", OP->NewPositionVector->VecData[j]);
            }
            printf("\n");
            printf("New velocity vector                       : ");
            for ( j = 0; j < NUMBER_OF_DOFS; j++)
            {
                printf("%10.3lf ", OP->NewVelocityVector->VecData[j]);
            }
            printf("\n");
            printf("New acceleration vector                   : ");
            for ( j = 0; j < NUMBER_OF_DOFS; j++)
            {
                printf("%10.3lf ", OP->NewAccelerationVector->VecData[j]);
            }
            printf("\n");
            printf("-------------------------------------------------------\n");
            printf("Extremes of the current trajectory:\n");

            for ( i = 0; i < NUMBER_OF_DOFS; i++)
            {
                printf("\n");
                printf("Degree of freedom                         : %d\n", i);
                printf("Minimum position                          : %10.3lf\n", OP->MinPosExtremaPositionVectorOnly->VecData[i]);
                printf("Time, at which the minimum will be reached: %10.3lf\n", OP->MinExtremaTimesVector->VecData[i]);
                printf("Position/pose vector at this time         : ");
                for ( j = 0; j < NUMBER_OF_DOFS; j++)
                {
                    printf("%10.3lf ", OP->MinPosExtremaPositionVectorArray[i]->VecData[j]);
                }
                printf("\n");
                printf("Velocity vector at this time              : ");
                for ( j = 0; j < NUMBER_OF_DOFS; j++)
                {
                    printf("%10.3lf ", OP->MinPosExtremaVelocityVectorArray[i]->VecData[j]);
                }
                printf("\n");
                printf("Acceleration vector at this time          : ");
                for ( j = 0; j < NUMBER_OF_DOFS; j++)
                {
                    printf("%10.3lf ", OP->MinPosExtremaAccelerationVectorArray[i]->VecData[j]);
                }
                printf("\n");
                printf("Maximum position                          : %10.3lf\n", OP->MaxPosExtremaPositionVectorOnly->VecData[i]);
                printf("Time, at which the maximum will be reached: %10.3lf\n", OP->MaxExtremaTimesVector->VecData[i]);
                printf("Position/pose vector at this time         : ");
                for ( j = 0; j < NUMBER_OF_DOFS; j++)
                {
                    printf("%10.3lf ", OP->MaxPosExtremaPositionVectorArray[i]->VecData[j]);
                }
                printf("\n");
                printf("Velocity vector at this time              : ");
                for ( j = 0; j < NUMBER_OF_DOFS; j++)
                {
                    printf("%10.3lf ", OP->MaxPosExtremaVelocityVectorArray[i]->VecData[j]);
                }
                printf("\n");
                printf("Acceleration vector at this time          : ");
                for ( j = 0; j < NUMBER_OF_DOFS; j++)
                {
                    printf("%10.3lf ", OP->MaxPosExtremaAccelerationVectorArray[i]->VecData[j]);
                }
                printf("\n");
            }
            printf("-------------------------------------------------------\n");
        */
        }

        // ****************************************************************

        // ****************************************************************
        // Feed the output values of the current control cycle back to
        // input values of the next control cycle

        *IP->CurrentPositionVector      =   *OP->NewPositionVector      ;
        *IP->CurrentVelocityVector      =   *OP->NewVelocityVector      ;
        *IP->CurrentAccelerationVector  =   *OP->NewAccelerationVector  ;
    }

    // ********************************************************************
    // Deleting the objects of the Reflexxes Motion Library end terminating
    // the process

    delete  RML         ;
    delete  IP          ;
    delete  OP          ;

    exit(EXIT_SUCCESS)  ;
}
