#include <QSystemSemaphore>

#include <iostream>
using namespace std;



int main(){
cout << "Initializing semaphore"<<endl;
QSystemSemaphore sem("camera");
sem.acquire();

while (true){
sem.release();
cout << "Processing Camera...."<<endl;
sleep(1);
sem.acquire();
cout << "Processing Worldmodel"<<endl;
sleep(1);
cout << "Processing Behavour"<<endl;
sleep(1);
cout << "Processing Comm"<<endl;
sleep(1);
cout << "Processing motion"<<endl;
sleep(1);
}








}

