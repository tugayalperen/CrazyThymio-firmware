/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"

#define DEBUG_MODULE "P2P"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "crtp_commander_high_level.h"
#include "commander.h"
#include "pptraj.h"
#include "estimator_kalman.h"
#include "deck_analog.h"

#define SEQUENCE_SPEED 1.0f
double var_a = 81;
double var_b = 50;
double angle = 0.0;
double r = 0.0;
double d = 0.0;
double grad = 0.0;
double grad_x = 0.0;
double grad_y = 0.0;
float temp_h = 0.0;
double grad_const_x = 25.0769;
double grad_const_y = 25.00;
double neg_xs[] = {0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0};
double neg_ys[] = {0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0};
double neg_zs[] = {0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0};
double neg_qs[] = {0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0};
double neg_hs[] = {0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0};
double self_pos[] = {0.0, 0.0, 0.0};
bool neg_alive[] = {false, false, false, false, false,
                    false, false, false, false, false};
double distance = 0.0;
double ij_ang = 0.0;
double ij_ang_local = 0.0;
double quadrant = 0.0;
double sensing_range = 2.0;
double dist_multiplier = 0;
double head_multiplier = 0;
int _fmode = 0;
bool quadrant_found = false;

float lmx = 900;
float lmn = 50;
float x_offset = -0.09;
float y_offset = 0.04;

double total_flight = 0.0;
static paramVarId_t paramIdCommanderEnHighLevel;
static paramVarId_t paramIdResetKalman;
static paramVarId_t paramIdiftakeoff;
static paramVarId_t paramIdifterminate;
static paramVarId_t paramIdifheading;
static paramVarId_t paramIdfmode;
static void resetKalman() { paramSetInt(paramIdResetKalman, 1); }
static void enableHighlevelCommander() { paramSetInt(paramIdCommanderEnHighLevel, 1); }

typedef struct {
  uint8_t id;
  float x;
  float y;
  float h;
  uint8_t l;
  } _coords;

typedef enum {
    idle,
    takingOff,
    onAir,
    land,
} State;

static State state = idle;


void p2pcallbackHandler(P2PPacket *p)
{
  //static TickType_t last_check=0;
  //TickType_t current_check = xTaskGetTickCount();

  _coords other_coords;
  memcpy(&other_coords, p->data, sizeof(other_coords));
  uint8_t other_id = other_coords.id;
  //for (int i = 0; i < 5; i++) 
  //  DEBUG_PRINT("B i: %i, Other ID: %i, neg_alive[i]: %s \n", i, other_id, neg_alive[i] ? "true" : "false"); 
  neg_alive[other_id - 1] = true;
  //for (int i = 0; i < 5; i++) 
  //  DEBUG_PRINT("A i: %i, Other ID: %i, neg_alive[i]: %s \n", i, other_id, neg_alive[i] ? "true" : "false"); 
  float other_X = other_coords.x;
  float other_Y = other_coords.y;
  float other_H = other_coords.h;
  neg_xs[other_id - 1] = (double)other_X;
  //DEBUG_PRINT("neg_xs:%f, neg_ys:%f - %lu\n", neg_xs[other_id-1], neg_ys[other_id-1], current_check - last_check );  
  neg_ys[other_id - 1] = (double)other_Y;
  neg_hs[other_id - 1] = (double)other_H;
  //last_check = current_check;
}

void appMain()
{
  // DEBUG_PRINT("Waiting for activation ...\n");

//    static setpoint_t setpoint;
    static float heading_log;
    heading_log = 99;
    static uint8_t iftakeoff;
    static uint8_t ifterminate;
    static uint8_t ifheading;
    static uint8_t smallest_distance[4];
    static uint8_t smallest_distance_hs[4];

    static float a_read = 0.0f;
    adcInit();
    static uint8_t light_log;
    light_log = 0;

    for (int i=0; i<4; i++)
    {
        smallest_distance[i] = 255;
        smallest_distance_hs[i] = 0;
    }

    dist_multiplier = 255 / sensing_range;
    head_multiplier = 255 / (3.141592*2);

    iftakeoff = 0;
    ifterminate = 0;
    ifheading = 1;

    logVarId_t idX = logGetVarId("stateEstimate", "x");
    logVarId_t idY = logGetVarId("stateEstimate", "y");
    logVarId_t idyaw = logGetVarId("stateEstimate", "yaw");

    static P2PPacket p_reply;
    p_reply.port=0x00;
    
    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    p2pRegisterCB(p2pcallbackHandler);
    _coords self_coords;
    self_coords.id = my_id;

    // logByFunction_t myLogger = {.aquireFloat = myLogValueFunction, .data = 0};
    LOG_GROUP_START(synthLog)
    LOG_ADD_CORE(LOG_UINT8, q1dist, &smallest_distance[0])
    LOG_ADD_CORE(LOG_UINT8, q2dist, &smallest_distance[1])
    LOG_ADD_CORE(LOG_UINT8, q3dist, &smallest_distance[2])
    LOG_ADD_CORE(LOG_UINT8, q4dist, &smallest_distance[3])
    LOG_ADD_CORE(LOG_UINT8, q1h, &smallest_distance_hs[0])
    LOG_ADD_CORE(LOG_UINT8, q2h, &smallest_distance_hs[1])
    LOG_ADD_CORE(LOG_UINT8, q3h, &smallest_distance_hs[2])
    LOG_ADD_CORE(LOG_UINT8, q4h, &smallest_distance_hs[3])
    LOG_ADD_CORE(LOG_UINT8, light_intensity, &light_log)
    LOG_GROUP_STOP(synthLog)

    PARAM_GROUP_START(fmodes)
    PARAM_ADD_CORE(PARAM_UINT8, if_takeoff, &iftakeoff)
    PARAM_ADD_CORE(PARAM_UINT8, if_terminate, &ifterminate)
    PARAM_ADD_CORE(PARAM_UINT8, if_heading, &ifheading)
    PARAM_GROUP_STOP(fmodes)

    paramIdCommanderEnHighLevel = paramGetVarId("commander", "enHighLevel");
    paramIdResetKalman = paramGetVarId("kalman", "resetEstimation");
    paramIdiftakeoff = paramGetVarId("fmodes", "if_takeoff");
    paramIdifterminate = paramGetVarId("fmodes", "if_terminate");
    resetKalman();
    enableHighlevelCommander();
    srand((unsigned int)xTaskGetTickCount());
    double test_rand = (double)rand()/(double)(RAND_MAX/6.28);
    DEBUG_PRINT("random is %f\n", test_rand);
    vTaskDelay(M2T(1000));


  // TickType_t time_begin=0, time_end=0;
  // const TickType_t LOOP_DURATION_TARGET = 50;

  while(1) {

    a_read = analogRead(DECK_GPIO_TX2);
    if(a_read>lmx)
    {a_read = lmx;}
    if(a_read<lmn)
    {a_read=lmn;}
    light_log = (uint8_t)((a_read-lmn)/(lmx-lmn)*255);
    self_coords.l = light_log;

    // time_begin = xTaskGetTickCount();
//     DEBUG_PRINT("lightlog_analog %f \n", (double)a_read);
    if (state == idle) {

      if (iftakeoff == 0) {
        iftakeoff = paramGetInt(paramIdiftakeoff);
        vTaskDelay(M2T(100));
      }

      if (iftakeoff == 1) {
        vTaskDelay(M2T(1000));
        // crtpCommanderHighLevelTakeoff(0.5, 2.5);
        // crtpCommanderHighLevelStartTrajectory(1, SEQUENCE_SPEED, true, false);
        vTaskDelay(M2T(1000));
        // vTaskDelay(M2T(5000));
        state = onAir;
        self_pos[2] = (double)rand()/(double)(RAND_MAX/6.28);
        // self_pos[2] = 0.0;
      }
    }

    // if (state == takingOff) {
    //   if (crtpCommanderHighLevelIsTrajectoryFinished()) 
    //   {state = onAir;}
    //   // state = onAir;
    // }

    if (state == onAir) {
      self_coords.x = logGetFloat(idX);
      self_coords.y = logGetFloat(idY);
//      self_coords.h = (float)self_pos[2];
      heading_log = self_coords.h;
      x_offset = cos(self_coords.h)*-0.09 - sin(self_coords.h)*0.04;
      y_offset = sin(self_coords.h)*-0.09 + cos(self_coords.h)*0.04;
      self_pos[2] = (double)self_coords.h;
      self_pos[0] = (double)(self_coords.x + x_offset);
      self_pos[1] = (double)(self_coords.y + y_offset);
      temp_h = logGetFloat(idyaw);
      self_coords.h = (double)temp_h * 0.0174532;

        grad_x = ceil(self_pos[0] * grad_const_x);
        grad_y = ceil(self_pos[1] * grad_const_y);
        if (grad_x >= 163) {grad_x = 162.0;}
        if (grad_x <= 0) {grad_x = 0.0;}
        if (grad_y >= 100) {grad_y = 99.0;}
        if (grad_y <= 0) {grad_y = 0;}
        angle = atan2(grad_y-50, grad_x-81);
        r = (var_a * var_b) / (sqrt(var_a*var_a*sin(angle)*sin(angle) + var_b*var_b*cos(angle)*cos(angle)));
        d = sqrt((grad_y - 50)*(grad_y - 50) + (grad_x-81)*(grad_x-81));
        grad = 255.0 - (d/r)*255.0;
        if (grad >= 255.0) {grad = 255.0;}
        if (grad <= 0.0) {grad = 0.0;}



      // DEBUG_PRINT("fmode: %i, ifheading: %i, goalX: %f, goalY: %f \n", _fmode, ifheading, (double)_goal_x, (double)_goal_y);
      // DEBUG_PRINT("K1: %f, K2: %f, alpha: %f, beta: %f, kappa: %f \n", K1, K2, alpha, beta, kappa);

      for (int i=0; i<4; i++)
      {
        smallest_distance[i] = 255;
        smallest_distance_hs[i] = 0;
      }

      for (int i = 0; i < 10; i++)
      {
        // DEBUG_PRINT("i: %i, My ID: %i, neg_alive[i]: %s \n", i, my_id, neg_alive[i] ? "true" : "false");
        quadrant_found = false;
        if ( neg_alive[i] && (neg_xs[i] > 0.0) && ((i+1) != my_id) )
        {
          // DEBUG_PRINT("i: %d\n", i);
          distance = sqrt( ((self_pos[0] - neg_xs[i])*(self_pos[0] - neg_xs[i])) + ((self_pos[1] - neg_ys[i])*(self_pos[1] - neg_ys[i])) ) - 0.3;
          if (distance < 0){
          distance = 0.0;}

          // DEBUG_PRINT("burada\n");
          ij_ang = atan2((neg_ys[i] - self_pos[1]), (neg_xs[i] - self_pos[0]));
          ij_ang_local = ij_ang - self_pos[2];
          ij_ang_local = atan2(sin(ij_ang_local), cos(ij_ang_local));

          if (ij_ang_local < 0.7854 && ij_ang_local >= -0.7854 && !(quadrant_found))
          {
            quadrant = 1;
            quadrant_found = true;
          }
          if (ij_ang_local < 2.3562 && ij_ang_local >= 0.7854 && !(quadrant_found))
          {
            quadrant = 2;
            quadrant_found = true;
          }
          if (ij_ang_local < 3.1416 && ij_ang_local >= 2.3562 && !(quadrant_found))
          {
            quadrant = 3;
            quadrant_found = true;
          }
          if (ij_ang_local < -0.7854 && ij_ang_local >= -2.3562 && !(quadrant_found))
          {
            quadrant = 4;
            quadrant_found = true;
          }

         neg_qs[i] = quadrant;
         if (distance > sensing_range)
         {distance = sensing_range;}

         if ((uint8_t)(distance*dist_multiplier) < smallest_distance[(int)quadrant-1])
         {
            smallest_distance[(int)quadrant-1] = (uint8_t)(distance*dist_multiplier);
            if (neg_hs[i] < 0)
            {
                neg_hs[i] = neg_hs[i] + (3.141592*2);
            }
            smallest_distance_hs[(int)quadrant-1] = (uint8_t)(head_multiplier*neg_hs[i]);
         }

//         if(print){
//         DEBUG_PRINT("Neg ID: %i, Neg X: %f, Neg Y: %f, Neg Quadrant: %f \n", i, neg_xs[i], neg_ys[i], neg_qs[i]);
//         DEBUG_PRINT("My heading: %f, IJang: %f, IJLocal: %f \n", self_pos[2], ij_ang, ij_ang_local);
//         }
//         print = ! print;

          // DEBUG_PRINT("Bearing is: %f\n", ij_ang);
        }
      }

      self_pos[2] = self_pos[2]; // 0.05s should be the loop time
      self_pos[2] = atan2(sin(self_pos[2]), cos(self_pos[2])); 
      
      // setHoverSetpoint(&setpoint, vx, vy, 0.5, 0.0);
      // commanderSetSetpoint(&setpoint, 3);  

      memcpy(p_reply.data, &self_coords, sizeof(self_coords));
      p_reply.size = sizeof(self_coords)+1;
      radiolinkSendP2PPacketBroadcast(&p_reply);
      ifterminate = paramGetInt(paramIdifterminate);
      _fmode = paramGetInt(paramIdfmode);
      ifheading = paramGetInt(paramIdifheading);
      // DEBUG_PRINT("fmode: %i \n", _fmode);
      vTaskDelay(M2T(50));
      // time_end = xTaskGetTickCount();
      // TickType_t loop_duration = time_end - time_begin;
      // if (loop_duration < LOOP_DURATION_TARGET ) {
        // vTaskDelay(M2T(LOOP_DURATION_TARGET - loop_duration));
      // } else {
        // DEBUG_PRINT("WARNING! loop took %lu ms, which is more than the target %lu ms\n", loop_duration, LOOP_DURATION_TARGET);
      // }
      total_flight += 50;     
    }

    if(state == land) {
      crtpCommanderHighLevelLand(0.03, 1.0);
      vTaskDelay(M2T(2000));
      return;
    }
  }
}

