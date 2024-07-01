/**
 * @file MessageHandler.h
 * @author icbaek
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once


#include <iostream>
#include <thread>
#include "MessageQueue.h"
#include "Message.h"


#define USE_SHARED_PTR 0

#if USE_SHARED_PTR == 1
    // shared_ptr 사용
    #define message_t                   std::shared_ptr<Message>
    #define SEND_MESSAGE     MSGHANDLE.sendMessage
    #define GET_MESSAGE_VAR(msg, arg)   msg->arg
#else
    // Message(type, arg)
    #define message_t                   Message
    #define SEND_MESSAGE                MSGHANDLE.sendMessage   // SEND_MESSAGE( message_t() )
    #define GET_MESSAGE_VAR(msg, arg)   msg.arg
#endif

#define MSGHANDLE CMessageHandler::getInstance()

class CMessageHandler 
{
private: /* not use! */
    CMessageHandler(const CMessageHandler& other) = default; // Copy constructor
    CMessageHandler& operator=(const CMessageHandler& other) = default; // Assignment operator

private:
    MessageQueue< message_t > mMessageQueue;

    CMessageHandler();
    ~CMessageHandler();

public:
    static CMessageHandler& getInstance();
    void sendMessage( message_t message);
    message_t dequeue();
    void clear();
    void destory();
};