#include <iostream>

// 使用 constexpr 进行编译期计算
constexpr int factorial(int n)
{
    return (n <= 1) ? 1 : (n * factorial(n - 1));
}

int main()
{
    constexpr int result = factorial(5);  // 编译期计算阶乘
    std::cout << "Factorial of 5: " << result << std::endl;
    return 0;
}
