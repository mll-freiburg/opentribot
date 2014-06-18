#include <qsharedmemory.h>

int main(){

QSharedMemory sharedMemory("foobar");
sharedMemory.create(1024);
sharedMemory.lock();
char *to = (char*)sharedMemory.data();
char *text = "hello world";
memcpy(to, text, strlen(text)+1);
sharedMemory.unlock();


}
