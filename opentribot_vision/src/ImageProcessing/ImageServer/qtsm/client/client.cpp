#include <qsharedmemory.h>
#include <qdebug.h>

int main(){

QSharedMemory sharedMemory("camera");
sharedMemory.attach();
sharedMemory.lock();
char *to = (char*)sharedMemory.data();
to[2]='a';
qDebug() << to;
sharedMemory.unlock();

}
