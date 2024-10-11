// 1.3 std::future 与 std::async
// std::future 与 std::async
// 提供了一种便捷的方式来获取异步操作的结果。std::async
// 启动一个异步任务，并返回一个 std::future 对象，允许在稍后获取结果。
#include <iostream>
#include <future>

int calculateSum(int a, int b)
{
    return a + b;
}

int main()
{
    std::future<int> result = std::async(calculateSum, 5, 10);  // 启动异步任务
    std::cout << "Sum: " << result.get() << std::endl;          // 获取结果
    return 0;
}
