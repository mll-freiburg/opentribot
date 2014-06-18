#include <QSystemSemaphore>

#include <iostream>

using namespace std;



int main(){

QSystemSemaphore sem("lala");
cout << sem.errorString().toStdString()<<endl;
//cout << "semaphore is available"<<endl;
while (true){
sem.acquire();
cout << "Processing."<<endl;
sleep(1);
sem.release();










}








}

