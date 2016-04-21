/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Sonar.cpp
 * Author: lukyh
 * 
 * Created on 27. března 2016, 19:50
 */

#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "Sonar.h"
#include "wiringPi.h"

pthread_mutex_t Sonar::muts1;
pthread_mutex_t Sonar::muts2;
pthread_mutex_t Sonar::muts3;
pthread_mutex_t Sonar::muts4;
float Sonar::front;
float Sonar::left;
float Sonar::right;
float Sonar::back;
int Sonar::set;

Sonar::Sonar() {
}

Sonar::Sonar(const Sonar& orig) {
}

Sonar::~Sonar() {
}

PI_THREAD(testThread){
    float echo, trigg;
    pthread_mutex_t *mutex;
    float *value;
    switch(Sonar::set){
        case 0:
            echo = 0;
            trigg = 1;
            mutex = &Sonar::muts1;
            value = &Sonar::front;
            break;
        case 1:
            echo = 2;
            trigg = 3;
            mutex = &Sonar::muts2;
            value = &Sonar::left;
            break;
        case 2:
            echo = 4;
            trigg = 5;
            mutex = &Sonar::muts3;
            value = &Sonar::right;
            break;
        case 3:
            echo = 6;
            trigg = 10;
            mutex = &Sonar::muts4;
            value = &Sonar::back;
            break;
        default:
            break; 
    }
    Sonar::set++;
    pinMode(echo, INPUT);
    pinMode(trigg, OUTPUT);
    delay(3);
    float old_length = 100.0f;
    while(1){
        loop:
        digitalWrite(trigg, LOW);
        delayMicroseconds(4);
        digitalWrite(trigg, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigg, LOW);
        unsigned int start_wait = micros();
        unsigned int dt;
        while(digitalRead(echo) == LOW){
            dt = micros();
            if(dt - start_wait > 60000) goto loop;
        }
        while(digitalRead(echo) == HIGH);
        float length = (old_length + (micros()-dt)/57.874)/2;
        pthread_mutex_lock(mutex);
        *value = length;
        pthread_mutex_unlock(mutex);
        old_length = length;
        delay(1);
    }
}

void Sonar::setUp(){
    set = 0;
    front = 100;
    left = 100;
    right = 100;
    back = 100;
    piThreadCreate(testThread);
    piThreadCreate(testThread);
    piThreadCreate(testThread);
    piThreadCreate(testThread);
}

