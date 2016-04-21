/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SerialComunication.h
 * Author: lukyh
 *
 * Created on 6. února 2016, 16:16
 */

#ifndef SERIALCOMUNICATION_H
#define SERIALCOMUNICATION_H
#include <wiringSerial.h>
#include <wiringPi.h>
#include <stdio.h>
#include "WebSocket.h"


struct accelerometer_data{int ax; int ay; int az;};
struct gyro_data{int gx; int gy; int gz;};
struct engine_data{int pitch; int roll; int yaw; int throttle;};
struct PID_struct{int S; int P; int I; int D;};
class WebSocket;
class SerialComunication {
public:
    SerialComunication();
    SerialComunication(const SerialComunication& orig);
    virtual ~SerialComunication();
    static void getSerialData();
    static bool setup();
    static void send();
    static int joinBytes(char firstByte, char value, int last_value);
    static struct engine_data engines;
    static struct accelerometer_data accel;
    static struct gyro_data gyro;
    static struct PID_struct PID;
    static void sendPID();
private:
    static int index_of_loop;
    static char values[100];
    static int handler;
};

#endif /* SERIALCOMUNICATION_H */

