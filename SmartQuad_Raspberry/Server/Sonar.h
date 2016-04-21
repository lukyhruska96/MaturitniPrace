/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Sonar.h
 * Author: lukyh
 *
 * Created on 27. března 2016, 19:50
 */

#ifndef SONAR_H
#define SONAR_H

class Sonar {
public:
    Sonar();
    Sonar(const Sonar& orig);
    virtual ~Sonar();
    static void setUp();
    static float front;
    static float left;
    static float right;
    static float back;
    static pthread_mutex_t muts1;
    static pthread_mutex_t muts2;
    static pthread_mutex_t muts3;
    static pthread_mutex_t muts4;
    static int set;
private:

};

#endif /* SONAR_H */

