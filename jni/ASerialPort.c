#include <unistd.h>
#include <termios.h>
#include <pthread.h>
#include "com_iamverylovely_ASerialPort.h"

static int nFd;
static JavaVM *vm;
static jobject jCallback;

void *comm_read_task(__unused void *args) {
    JNIEnv *_env;
    (*vm)->AttachCurrentThread(vm, &_env, NULL);
    jclass jClass1 = (*_env)->GetObjectClass(_env, jCallback);
    jmethodID jReceive = (*_env)->GetMethodID(_env, jClass1, "OnReceive", "([BI)V");
    char read_buf[256];
    while (1) {
        if (nFd == -1) {
            (*_env)->DeleteLocalRef(_env, jReceive);
            (*_env)->DeleteLocalRef(_env, jClass1);
            (*_env)->DeleteGlobalRef(_env, jCallback);
            (*vm)->DetachCurrentThread(vm);
            break;
        }
        int nLen = (int) read(nFd, &read_buf, sizeof(read_buf));
        if (nLen == -1) {
            nFd = -1;
            continue;
        }
        jbyteArray jByteArr = (*_env)->NewByteArray(_env, nLen);
        (*_env)->SetByteArrayRegion(_env, jByteArr, 0, nLen, (jbyte *) read_buf);
        (*_env)->CallVoidMethod(_env, jCallback, jReceive, jByteArr, nLen);
        (*_env)->DeleteLocalRef(_env, jByteArr);
    }
    return 0;
}

JNIEXPORT jint JNICALL
Java_com_iamverylovely_ASerialPort_reading
        (__unused JNIEnv *env, __unused jclass clazz, jint fd, jint speed, jobject callback) {
    if (fd == -1) {
        return -1;
    }

    struct termios cfg;
    if (tcgetattr(fd, &cfg)) {
        close(fd);
        return -1;
    }

    cfmakeraw(&cfg);
    cfsetispeed(&cfg, 0);
    cfsetospeed(&cfg, speed);

    if (tcsetattr(fd, TCSANOW, &cfg)) {
        close(fd);
        return -1;
    }

    nFd = fd;

    jboolean isNull = (*env)->IsSameObject(env, NULL, callback);
    if (isNull) {
        return 0;
    }

    (*env)->GetJavaVM(env, &vm);
    jCallback = (*env)->NewGlobalRef(env, callback);

    pthread_t pid;
    pthread_create(&pid, 0, comm_read_task, NULL);
    return nFd;
}

JNIEXPORT void JNICALL
Java_com_iamverylovely_ASerialPort_send(JNIEnv *env, __unused jclass clazz,
                                        jbyteArray data) {
    int nLen = (*env)->GetArrayLength(env, data);
    unsigned char *pData[nLen];
    (*env)->GetByteArrayRegion(env, data, 0, nLen, (jbyte *) pData);
    write(nFd, pData, nLen);
    (*env)->DeleteLocalRef(env, data);
}

JNIEXPORT void JNICALL
Java_com_iamverylovely_ASerialPort_close(__unused JNIEnv *env, __unused jclass clazz) {
    close(nFd);
}