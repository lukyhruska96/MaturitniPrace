/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: lukyh
 *
 * Created on 5. února 2016, 19:07
 */

#include "SerialComunication.h"
#include "WebSocket.h"
#include "Sonar.h"
#include <stdio.h>
#include <pthread.h>
#include <iostream>

using namespace std;

pthread_mutex_t muts1;

int main(int argc, char** argv) {
    if(!SerialComunication::setup()) return 0;
    WebSocket::setUp();
    Sonar::setUp();
    while(1){
        SerialComunication::getSerialData();
        WebSocket::service();
        //printf("%d\t%d\t%d\t%d\t%d\t%d\n", ax, ay, az, gx, gy, gz);
    }
    WebSocket::destroy();
    return 0;
}
