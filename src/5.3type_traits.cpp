#include <iostream>
#include <type_traits>

int main()
{
    std::cout << std::boolalpha;  // 输出布尔值为 true/false 形式

    // 检查类型是否为整数类型
    std::cout << "Is int an integral type? " << std::is_integral<int>::value
              << std::endl;
    std::cout << "Is float an integral type? " << std::is_integral<float>::value
              << std::endl;

    // 检查类型是否为浮点数类型
    std::cout << "Is double a floating-point type? "
              << std::is_floating_point<double>::value << std::endl;

    return 0;
}
