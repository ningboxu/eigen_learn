// 1.1 std::thread
// C++11 引入了标准线程库，std::thread
// 是用于创建并管理线程的类。线程允许程序并行执行多个任务。
#include <iostream>
#include <thread>

// 线程函数
void printMessage()
{
    std::cout << "Hello from thread!" << std::endl;
}

int main()
{
    std::thread t(printMessage);  // 创建线程并执行
    t.join();                     // 等待线程执行完毕
    return 0;
}
