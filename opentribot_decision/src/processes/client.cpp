#include <qsharedmemory.h>
#include <qdebug.h>

int main(){

QSharedMemory sharedMemory("foobar");
sharedMemory.attach();
sharedMemory.lock();
char *to = (char*)sharedMemory.data();
qDebug() << to;
sharedMemory.unlock();

}
