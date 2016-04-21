/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   WebSocket.h
 * Author: lukyh
 *
 * Created on 6. února 2016, 17:10
 */

#ifndef WEBSOCKET_H
#define WEBSOCKET_H
#include <libwebsockets.h>
#include "SerialComunication.h"
#include <pthread.h>
#include <string.h>

    extern int callback_dumb_increment(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len);
    extern int callback_http(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len);
       

    
static const struct lws_extension exts[] = {
	{
		"permessage-deflate",
		lws_extension_callback_pm_deflate,
		"permessage-deflate"
	},
	{
		"deflate-frame",
		lws_extension_callback_pm_deflate,
		"deflate_frame"
	},
	{ NULL, NULL, NULL /* terminator */ }
};
class SerialComunication;
class WebSocket {
public:
    WebSocket();
    WebSocket(const WebSocket& orig);
    struct per_session_data__http {lws_filefd_type fd;};
    struct per_session_data__dumb_increment{int number;};
    static void service();
    static void destroy();
    static void setUp();
    static void sendAll(char* message, int len, lws *wsi=NULL);
    virtual ~WebSocket();
    static unsigned int num_connection; 
    static lws* sockets[100];
    static lws* controller;
    static char passwd[32];
    static struct lws_context *context;
private:
    static struct lws_context_creation_info info;
};

static struct lws_protocols protocols[] = {
	/* first protocol must always be HTTP handler */
	{
		"http-only",		/* name */
		callback_http,		/* callback */
		sizeof(struct WebSocket::per_session_data__http),	/* per_session_data_size */
		0,			/* max frame size / rx buffer */
	},
	{
		"dumb-increment-protocol",
		callback_dumb_increment,
		sizeof(struct WebSocket::per_session_data__dumb_increment),
		4000,
	},
	{ NULL, NULL, 0, 0 }
};

#endif /* WEBSOCKET_H */

