/*********************************************************************
 *                      Apache License
 *                 Version 2.0, January 2004
 *               http://www.apache.org/licenses/
 *
 * tm_otg.cpp
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

#include "tm_reflexxes/include/tm_reflexxes/tm_reflexxes.h"


//*************************************************************************
// defines

#define CYCLE_TIME_IN_SECONDS                   0.025
#define NUMBER_OF_DOFS                          6
#define DEG2RAD 0.01745329252
#define RAD2DEG 57.29577951

#define MAX_VELOCITY 1.0
#define MAX_ACC 0.0375*40 // 0.0375 : acc in 25ms

using namespace std;

int main(int argc, char **argv)
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

   /* while(run_succeed)
    {
        if (run_succeed)
        {
            TargetPosition = {0,0,0,0,1.57,0};
            TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
            run_succeed = tm_reflexxes::ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        }
        else
            break;

        if (run_succeed)
        {
            TargetPosition = {0,0,0,0,0,0};
            TargetVelocity = {0.0, 0.0, 0.0, 0.0, 0.0};
            run_succeed = tm_reflexxes::ReflexxesPositionRun_sim(*IP_position, TargetPosition, TargetVelocity, SynchronousTime);
        }
        else
            break;
    }*/

    
    while(run_succeed)
    {
        if(run_succeed)
        {
            TargetVelocity = {0,0,0,0,1.0,0};
            run_succeed = tm_reflexxes::ReflexxesVelocityRun_sim(*IP_velocity, TargetVelocity, SynchronousTime);
        }
        else
            break;
        if (run_succeed)
        {
            TargetVelocity = {0,0,0,0,-1.0,0};
            run_succeed = tm_reflexxes::ReflexxesVelocityRun_sim(*IP_velocity, TargetVelocity, SynchronousTime);
        }
        else
            break;
    }



    delete IP_position;
    delete IP_velocity;

    return 0;
}




