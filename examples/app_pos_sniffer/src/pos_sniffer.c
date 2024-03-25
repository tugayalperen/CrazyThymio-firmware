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
#include "estimator_kalman.h"

#define SEQUENCE_SPEED 1.0f
double neg_xs[] = {0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0};
double neg_ys[] = {0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0};
double neg_ls[] = {0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0};
double neg_hs[] = {0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0};
bool neg_alive[] = {false, false, false, false, false,
                    false, false, false, false, false};
int32_t neg_states[] = {0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0};
float lmx = 900;
float lmn = 50;

double total_flight = 0.0;

int32_t aggregate_data(double x, double y, double h, double l)
{
  /*
  Aggregation function to send relevant data in a single int.
  Example: x:5.0 y:1.0 heading(yaw):2.5 light_intensity:120
  This becomes -> 501025120
  */
  if (h < 0)
  {
    h = h + 6.28;
  }

  x = round(x * 10) / 10;
  y = round(y * 10) / 10;
  h = round(h * 10) / 10;
  l = round(l * 10) / 10;

  x = x * pow(10, 8);
  y = y * pow(10, 6);
  h = h * pow(10, 4);

  double t;
  t = x + y + h + l;

  int32_t c_data_aggregated;
  c_data_aggregated = (int32_t)t;

  return c_data_aggregated;
}

typedef struct {
  uint8_t id;
  float x;
  float y;
  float h;
  uint8_t l;
  } _coords;

void p2pcallbackHandler(P2PPacket *p)
{
  _coords other_coords;
  memcpy(&other_coords, p->data, sizeof(other_coords));
  uint8_t other_id = other_coords.id;
  neg_alive[other_id - 1] = true;
  float other_X = other_coords.x;
  float other_Y = other_coords.y;
  float other_H = other_coords.h;
  uint8_t other_L = other_coords.l;
  neg_xs[other_id - 1] = (double)other_X;
  neg_ys[other_id - 1] = (double)other_Y;
  neg_hs[other_id - 1] = (double)other_H;
  neg_ls[other_id - 1] = (double)other_L*1;
}

void appMain()
{

    static P2PPacket p_reply;
    p_reply.port=0x00;
    
    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    p2pRegisterCB(p2pcallbackHandler);
    _coords self_coords;
    self_coords.id = my_id;

    // logByFunction_t myLogger = {.aquireFloat = myLogValueFunction, .data = 0};
    LOG_GROUP_START(synthLog)
    LOG_ADD_CORE(LOG_INT32, CF1, &neg_states[0])
    LOG_ADD_CORE(LOG_INT32, CF2, &neg_states[1])
    LOG_GROUP_STOP(synthLog)


    srand((unsigned int)xTaskGetTickCount());
    double test_rand = (double)rand()/(double)(RAND_MAX/6.28);
    DEBUG_PRINT("random is %f\n", test_rand);
    vTaskDelay(M2T(1000));


  // TickType_t time_begin=0, time_end=0;
  // const TickType_t LOOP_DURATION_TARGET = 50;
  while(1) {

      for (int i = 0; i < 10; i++)
      {
        neg_states[i] = aggregate_data(neg_xs[i], neg_ys[i], neg_hs[i], neg_ls[i]);
      }
      vTaskDelay(M2T(50));
      total_flight += 50;

      }


//     DEBUG_PRINT("Neg ID: %i, Neg X: %f, Neg Y: %f, Neg Quadrant: %f \n", i, neg_xs[i], neg_ys[i], neg_qs[i]);
//     DEBUG_PRINT("My heading: %f, IJang: %f, IJLocal: %f \n", self_pos[2], ij_ang, ij_ang_local);
      // DEBUG_PRINT("Bearing is: %f\n", ij_ang);





    }
