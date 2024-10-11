#include <iostream>
#include <optional>  //c++17

std::optional<int> findNumber(bool found)
{
    if (found)
    {
        return 42;  // 返回有效值
    }
    else
    {
        return {};  // 返回空值
    }
}

int main()
{
    std::optional<int> result = findNumber(true);

    if (result)
    {
        std::cout << "Found number: " << result.value() << std::endl;
    }
    else
    {
        std::cout << "Number not found" << std::endl;
    }

    return 0;
}
