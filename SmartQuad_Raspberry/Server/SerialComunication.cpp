/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   SerialComunication.cpp
 * Author: lukyh
 * 
 * Created on 6. února 2016, 16:16
 */

#include "SerialComunication.h"
#include "WebSocket.h"
#include "Sonar.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <wiringSerial.h>
#include <libwebsockets.h>
#include <string>
#include <wiringPi.h>

int SerialComunication::index_of_loop;
char SerialComunication::values[100];
int SerialComunication::handler;
struct engine_data SerialComunication::engines;
struct accelerometer_data SerialComunication::accel;
struct gyro_data SerialComunication::gyro;
struct PID_struct SerialComunication::PID;

SerialComunication::SerialComunication() {
    setup();
}

SerialComunication::SerialComunication(const SerialComunication& orig) {
}

SerialComunication::~SerialComunication() {
}

void SerialComunication::getSerialData(){
    printf("SERIAL\n");
        if(!serialDataAvail(handler)) return;
        if(serialDataAvail(handler) > 32)
            while(serialDataAvail(handler) != 32) char tmp = serialGetchar(handler);
        while(serialDataAvail(handler)){
            values[index_of_loop] = serialGetchar(handler);
            index_of_loop++;
        }
        if(index_of_loop < 32) return;
        int num_syncs = 0;
        int i;
        for(i = 0; i<index_of_loop; ++i){
            if(values[i] == 0xFE) num_syncs++;
            else num_syncs = 0;
            if(num_syncs == 4) break;
        }
        if(num_syncs != 4) return;  
        accel.ax = joinBytes(values[i+1], values[i+2], accel.ax);
        accel.ay = joinBytes(values[i+3], values[i+4], accel.ay);
        accel.az = joinBytes(values[i+5], values[i+6], accel.az);
        gyro.gx = joinBytes(values[i+7], values[i+8], gyro.gx);
        gyro.gy = joinBytes(values[i+9], values[i+10], gyro.gy);
        gyro.gz = joinBytes(values[i+11], values[i+12], gyro.gz);
        printf("RECEIVED DATA\n");
        char *string = (char *)malloc(61);
        pthread_mutex_lock(&Sonar::muts1);
        pthread_mutex_lock(&Sonar::muts2);
        pthread_mutex_lock(&Sonar::muts3);
        pthread_mutex_lock(&Sonar::muts4);
        sprintf(string, "coordinate;%4d;%4d;%4d;%4d;%4d;%4d;%4d;%4d;%4d;%4d", accel.ax, accel.ay, \
                accel.az, gyro.gx, gyro.gy, gyro.gz, (int)Sonar::front, (int)Sonar::left, (int)Sonar::right, (int)Sonar::back);
        lws_callback_on_writable_all_protocol(WebSocket::context, protocols);
        WebSocket::sendAll(string, 61);
        printf("SENT TO WEBSOCKET\n");
        printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", accel.ax, accel.ay, \
                accel.az, gyro.gx, gyro.gy, gyro.gz, (int)Sonar::front, (int)Sonar::left, (int)Sonar::right, (int)Sonar::back);
        pthread_mutex_unlock(&Sonar::muts1);
        pthread_mutex_unlock(&Sonar::muts2);
        pthread_mutex_unlock(&Sonar::muts3);
        pthread_mutex_unlock(&Sonar::muts4);
        index_of_loop = 0;
        serialFlush(handler);
        if(Sonar::left < 90 || Sonar::right < 90){
                        if(Sonar::left < 90 && Sonar::right < 90) SerialComunication::engines.roll = Sonar::left/90.0*20.0 - Sonar::right/90.0-20.0;
                        else{
                            if(Sonar::left < 90) SerialComunication::engines.roll = Sonar::left/90.0*20.0;
                            else SerialComunication::engines.roll = Sonar::right/90.0*(-20.0);
                        }
                    }
        if(Sonar::front < 90 || Sonar::back < 90){
                        if(Sonar::front < 90 && Sonar::back < 90) SerialComunication::engines.pitch = Sonar::back/90.0*20.0 - Sonar::front/90.0-20.0;
                        else{
                            if(Sonar::back < 90) SerialComunication::engines.pitch = Sonar::back/90.0*20.0;
                            else SerialComunication::engines.pitch = Sonar::front/90.0*(-20.0);
                        }
                    }
        send();
}

bool SerialComunication::setup(){
    index_of_loop = 0;
    if (wiringPiSetup())
    {
        printf ("Unable to initialise wiringPi\n") ;
        return false;
    }
    else{
        printf("WIRING OK\n");
    }
    handler =  serialOpen("/dev/ttyAMA0", 115200);
    if(handler == -1){
        printf("Problem with opening serialPort\n");
        return false;
    }
    return true;
}

int SerialComunication::joinBytes(char firstByte, char value, int last_value){
    if(firstByte == 0xFF) return value*(-1);
    else return value;
}

void SerialComunication::send(){
    printf("SERIAL SEND\n");
    char *message = (char *)malloc(9);
    message[0] = (char)0xFE;
    message[1] = (char)0x00;
    message[2] = (char)abs(engines.pitch);
    if(engines.pitch>=0)message[3] = (char)0x00;
    else message[3] = (char)0xFF;
    message[4] = (char)abs(engines.roll);
    if(engines.roll>=0)message[5] = (char)0x00;
    else message[5] = (char)0xFF;
    message[6] = (char)abs(engines.yaw);
    if(engines.yaw>=0)message[7] = (char)0x00;
    else message[7] = (char)0xFF;
    message[8] = (char)abs(engines.throttle);
    for(int i = 0; i<9; i++){
        printf("%x\t", message[i]);
    }
    printf("\n");
    for(int i = 0; i<9; i++){
        serialPutchar(SerialComunication::handler, (unsigned char)message[i]);
    }
    printf("SERIAL SENT\n");
    free(message);
}

void SerialComunication::sendPID(){
    char *message = (char *)malloc(6);
    message[0] = (char)0xFE;
    message[1] = (char)0x01;
    message[2] = (char)PID.S;
    message[3] = (char)PID.P;
    message[4] = (char)PID.I;
    message[5] = (char)PID.D;
    for(int i = 0; i<6; i++){
        serialPutchar(SerialComunication::handler, (unsigned char)message[i]);
    }
    free(message);
}