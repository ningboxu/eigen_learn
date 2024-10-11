#include <iostream>
#include <tuple>

int main()
{
    // 创建 tuple，包含不同类型的元素
    std::tuple<int, double, std::string> myTuple =
        std::make_tuple(1, 3.14, "Hello");

    // 获取 tuple 中的元素
    std::cout << "Int: " << std::get<0>(myTuple) << std::endl;
    std::cout << "Double: " << std::get<1>(myTuple) << std::endl;
    std::cout << "String: " << std::get<2>(myTuple) << std::endl;

    return 0;
}
