#include <iostream>

// 编译期计算斐波那契数列
constexpr int fibonacci(int n)
{
    return (n <= 1) ? n : (fibonacci(n - 1) + fibonacci(n - 2));
}

int main()
{
    constexpr int fib_10 = fibonacci(10);  // 编译期计算 Fibonacci(10)
    std::cout << "Fibonacci of 10: " << fib_10 << std::endl;
    return 0;
}
