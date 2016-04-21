/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   WebSocket.cpp
 * Author: lukyh
 * 
 * Created on 6. února 2016, 17:10
 */

#include <libwebsockets.h>

#include "WebSocket.h"
#include "Sonar.h"


struct lws_context *WebSocket::context;
struct lws_context_creation_info WebSocket::info;
unsigned int WebSocket::num_connection; 
lws* WebSocket::sockets[100];
char WebSocket::passwd[32];
lws* WebSocket::controller;

struct send_args {lws *wsi; char* message; int len;};

static void *sendThread(void *args){
    struct send_args *sent_args = (struct send_args *) args;
    if(lws_partial_buffered (sent_args->wsi)!=1){
    lws_write(sent_args->wsi,(unsigned char*)sent_args->message, sent_args->len, LWS_WRITE_TEXT);
    free(sent_args);
    pthread_exit(NULL);
    }
}

WebSocket::WebSocket() {
    setUp();
}

void WebSocket::setUp(){
    controller = NULL;
    strcpy(passwd,"5e6c74f203479ec32aac3d88f27504be");
    num_connection = 0;
    for(int i = 0; i<100; i++)
        sockets[i] = NULL;
    int opts = 0;
    info.port = 8080;
    info.iface = NULL;
    info.protocols = protocols;
    info.ssl_cert_filepath = NULL;
    info.ssl_private_key_filepath = NULL;
    info.gid = -1;
    info.uid = -1;
    info.max_http_header_pool = 1;
    info.options = opts | LWS_SERVER_OPTION_VALIDATE_UTF8;
    info.extensions = exts;
    context = lws_create_context(&info);
    if (context == NULL) {
		lwsl_err("libwebsocket init failed\n");
	}
}

WebSocket::WebSocket(const WebSocket& orig) {
}

WebSocket::~WebSocket() {
}

void WebSocket::service(){
        lws_service(context, 50);
}

void WebSocket::destroy(){
    lws_context_destroy(context);
}

void WebSocket::sendAll(char* message, int len, lws *wsi){
    pthread_t t[num_connection];
    for(int i = 0; i<num_connection; i++){
        if(wsi != NULL && sockets[i] == wsi) continue;
        struct send_args *args = (struct send_args *)malloc(sizeof(struct send_args));
        args->len = len;
        args->message = message;
        if(sockets[i] != NULL){
        args->wsi = sockets[i];
        int rc = pthread_create(&t[i], NULL, &sendThread, (void *)args);
        if(rc)printf("can't create new thread!\n");
        }
    }
    for(int i = 0;i<num_connection;i++)
{
        if(sockets[i] == wsi) continue;
    pthread_join(t[i],NULL);
}
    free(message);
}

extern int callback_dumb_increment(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len){
    printf("WEBSOCKET SERVE\n");
    switch(reason){
        case LWS_CALLBACK_ESTABLISHED:{
            printf("Conection estabilished\n");
            lws_callback_on_writable_all_protocol(WebSocket::context, protocols);
            char *message = (char *)malloc(23);
            sprintf(message, "engines;%3d;%3d;%3d;%3d", SerialComunication::engines.pitch, SerialComunication::engines.roll, \
                    SerialComunication::engines.yaw, SerialComunication::engines.throttle);
            lws_write(wsi, (unsigned char*)message, 23, LWS_WRITE_TEXT);
            WebSocket::sockets[WebSocket::num_connection] = wsi;
            WebSocket::num_connection++;
            free(message);
            break;
        }
        case LWS_CALLBACK_RECEIVE:
        {
            printf("RECEIVED\n");
            lws_callback_on_writable_all_protocol(WebSocket::context, protocols);
            int size_instruct = 0;
            char *buf = (char *) in;
            while(buf[size_instruct] != ';' && size_instruct < len) size_instruct++;
            char *instruct = (char *) malloc(size_instruct);
            strncpy(instruct, buf, size_instruct);
            if(size_instruct == 7 && wsi == WebSocket::controller){
                printf("ENGINES\n");
                    int last_size = size_instruct+1;
                    int length = 0;
                    while(buf[last_size+length]!=';') length++;
                    char *tmp1 = (char *)malloc(length);
                    strncpy(tmp1, &buf[last_size], length);
                    last_size += length+1;
                    length = 0;
                    while(buf[last_size+length]!=';') length++;
                    char *tmp2 = (char *)malloc(length);
                    strncpy(tmp2, &buf[last_size], length);
                    last_size += length+1;
                    length = 0;
                    while(buf[last_size+length]!=';') length++;
                    char *tmp3 = (char *)malloc(length);
                    strncpy(tmp3, &buf[last_size], length);
                    last_size += length+1;
                    length = 0;
                    while(buf[last_size+length]!=';') length++;
                    char *tmp4 = (char *)malloc(length);
                    strncpy(tmp4, &buf[last_size], length);
                    
                    SerialComunication::engines.pitch = atoi(tmp1);
                    SerialComunication::engines.roll = atoi(tmp2);
                    SerialComunication::engines.yaw = atoi(tmp3);
                    SerialComunication::engines.throttle = atoi(tmp4);
                    printf("Received: %d\t%d\t%d\t%d\t\n", SerialComunication::engines.pitch,\
                            SerialComunication::engines.roll, SerialComunication::engines.yaw, \
                            SerialComunication::engines.throttle);
                    char *message = (char *)malloc(23);
                    sprintf(message, "engines;%3d;%3d;%3d;%3d", SerialComunication::engines.pitch, SerialComunication::engines.roll, \
                            SerialComunication::engines.yaw, SerialComunication::engines.throttle);
                    WebSocket::sendAll(message, 23, wsi);
                    free(tmp1);
                    free(tmp2);
                    free(tmp3);
                    free(tmp4);
            }
            else if(size_instruct == 6){
                printf("VERIFY\n");
                char *message = (char *)malloc(8);
                if(WebSocket::controller != NULL){
                    strcpy(message, "verify;2");
                    lws_write(wsi, (unsigned char *)message, 8, LWS_WRITE_TEXT);
                    free(message);
                    break;
                }
                char md5[32];
                strncpy(md5, buf+7, 32);
                bool same = true;
                for(int i = 0; i<32; i++){
                    if(md5[i] != WebSocket::passwd[i]) same = false;
                }
                if(same){
                    WebSocket::controller = wsi;
                    strcpy(message, "verify;1");
                    lws_write(wsi, (unsigned char *)message, 8, LWS_WRITE_TEXT);
                }
                else{
                    strcpy(message, "verify;0");
                    lws_write(wsi, (unsigned char *)message, 8, LWS_WRITE_TEXT);
                }
                free(message);
            }
            else if(size_instruct == 3 && wsi == WebSocket::controller){
                printf("PID\n");
                int last_size = size_instruct+1;
                int length = 0;
                while(buf[last_size+length]!=';') length++;
                char *tmp1 = (char *)malloc(length);
                strncpy(tmp1, &buf[last_size], length);
                last_size += length+1;
                length = 0;
                while(buf[last_size+length]!=';') length++;
                char *tmp2 = (char *)malloc(length);
                strncpy(tmp2, &buf[last_size], length);
                last_size += length+1;
                length = 0;
                while(buf[last_size+length]!=';') length++;
                char *tmp3 = (char *)malloc(length);
                strncpy(tmp3, &buf[last_size], length);
                last_size += length+1;
                length = 0;
                while(buf[last_size+length]!=';') length++;
                char *tmp4 = (char *)malloc(length);
                strncpy(tmp4, &buf[last_size], length);
                last_size += length+1;
                length = 0;
                SerialComunication::PID.S = atoi(tmp1);
                SerialComunication::PID.P = atoi(tmp2);
                SerialComunication::PID.I = atoi(tmp3);
                SerialComunication::PID.D = atoi(tmp4);
                SerialComunication::sendPID();
                printf("Sent PID\n");
                free(tmp1);
                free(tmp2);
                free(tmp3);
                free(tmp4);
            }
            free(instruct);
            break;
        }
        case LWS_CALLBACK_CLOSED:
        {
            lws_callback_on_writable_all_protocol(WebSocket::context, protocols);
            printf("Connection Closed\n");
            if(wsi == WebSocket::controller){
                char *message = (char *)malloc(8);
                strcpy(message, "verify;3");
                WebSocket::sendAll(message, 8);
                WebSocket::controller = NULL;
            }
            for(int i = 0; i<WebSocket::num_connection; i++){
                if(WebSocket::sockets[i] == wsi){
                    for(int y = i; y<WebSocket::num_connection-1; y++){
                        WebSocket::sockets[y] = WebSocket::sockets[y+1];
                    }
                    break;
                }
            }
            WebSocket::num_connection--;
            break;
        }
        default:
            break;
    }
    return 0;
}

extern int callback_http(lws* wsi, lws_callback_reasons reason, void* user, void* in, size_t len){
    return 0;
}
extern int callback_lws_echogen(struct lws *wsi, enum lws_callback_reasons reason,
			void *user, void *in, size_t len);

extern void dump_handshake_info(struct lws *wsi);

