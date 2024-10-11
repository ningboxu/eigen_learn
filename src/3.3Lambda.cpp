#include <iostream>
#include <vector>
#include <algorithm>

int main()
{
    std::vector<int> numbers = {1, 2, 3, 4, 5};

    // 使用 lambda 表达式进行求和
    int sum = 0;
    std::for_each(numbers.begin(), numbers.end(), [&sum](int x) { sum += x; });

    std::cout << "Sum: " << sum << std::endl;

    // 简单的 lambda 表达式返回值
    auto add = [](int a, int b) -> int { return a + b; };

    std::cout << "5 + 10 = " << add(5, 10) << std::endl;

    return 0;
}
