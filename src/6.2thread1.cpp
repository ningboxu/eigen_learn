// 1.2 传递参数给线程
// 线程函数可以接受参数，参数可以通过值或引用传递。
#include <iostream>
#include <thread>

void printSum(int a, int b)
{
    std::cout << "Sum: " << (a + b) << std::endl;
}

int main()
{
    std::thread t(printSum, 5, 10);  // 传递参数
    t.join();
    return 0;
}
