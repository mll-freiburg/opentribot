#include <qsharedmemory.h>
#include <iostream>



int main(){

QSharedMemory sharedMemory("camera");
sharedMemory.create(1024*1024*3*4);
sharedMemory.lock();
char *to = (char*)sharedMemory.data();
char *text = "hello world";
memcpy(to, text, strlen(text)+1);
sharedMemory.unlock();
char a;
std::cin>>a; ;

}
