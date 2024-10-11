// SFINAE（Substitution Failure Is Not An Error）
// 概念：
// SFINAE 是 C++
// 模板中的一个规则，当模板实参替换失败时，它不会导致编译错误，而是会尝试寻找其他重载或特化版本。这允许你编写更加灵活的模板代码。

// 通过 SFINAE 选择合适的函数模板重载。
#include <iostream>
#include <type_traits>

// 通用版本
template <typename T>
typename std::enable_if<std::is_integral<T>::value>::type checkType(T value)
{
    std::cout << value << " is an integer." << std::endl;
}

// 另一版本
template <typename T>
typename std::enable_if<std::is_floating_point<T>::value>::type checkType(
    T value)
{
    std::cout << value << " is a floating-point number." << std::endl;
}

int main()
{
    checkType(42);    // 调用整数版本
    checkType(3.14);  // 调用浮点数版本

    return 0;
}
