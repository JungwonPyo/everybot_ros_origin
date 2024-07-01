
#pragma once
#include <pthread.h>

class CPthreadLockGuard {
public:
    explicit CPthreadLockGuard(pthread_mutex_t &mutex) : m_mutex(mutex) {
        pthread_mutex_lock(&m_mutex); // 생성자에서 뮤텍스 잠금
    }

    ~CPthreadLockGuard() {
        pthread_mutex_unlock(&m_mutex); // 소멸자에서 뮤텍스 해제
    }

    // 복사 생성자와 복사 대입 연산자를 삭제하여 복사를 방지
    CPthreadLockGuard(const CPthreadLockGuard&) = delete;
    CPthreadLockGuard& operator=(const CPthreadLockGuard&) = delete;

private:
    pthread_mutex_t &m_mutex;
};

/*
// 사용 예시
pthread_mutex_t mutex;
pthread_mutex_init(&mutex, nullptr); // 뮤텍스 초기화

void someFunction() {
    CPthreadLockGuard lock(mutex); // 함수 내에서 자동으로 뮤텍스 관리
    // 안전한 영역 - 뮤텍스로 보호됨
}

// 프로그램 종료 시 뮤텍스 정리
pthread_mutex_destroy(&mutex);
*/