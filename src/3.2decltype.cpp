#include <iostream>

int main() {
    int a = 5;
    decltype(a) b = 10;  // 自动推断 b 的类型为 int
    std::cout << "b: " << b << std::endl;

    decltype(a + b) c = 15;  // 推断 c 的类型为 int
    std::cout << "c: " << c << std::endl;

    return 0;
}
