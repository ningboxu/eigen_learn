#include <iostream>
#include <vector>

int main()
{
    auto i               = 10;    // 自动推断为 int
    auto d               = 3.14;  // 自动推断为 double
    std::vector<int> vec = {1, 2, 3, 4, 5};

    // 使用 auto 推断迭代器类型
    for (auto it = vec.begin(); it != vec.end(); ++it)
    {
        std::cout << *it << " ";
    }
    std::cout << std::endl;

    return 0;
}
