#include <iostream>
#include <vector>
// 右值引用（&&）是 C++11引入的特性，用于优化对象的移动操作，避免不必要的拷贝。
// 移动语义允许对象的资源直接从一个对象“转移”到另一个对象，而不是拷贝数据。
class MyClass
{
public:
    std::vector<int> data;

    // 构造函数
    MyClass(std::vector<int> d) : data(std::move(d))
    {
        std::cout << "Move constructor called!" << std::endl;
    }
};

int main()
{
    std::vector<int> vec = {1, 2, 3, 4, 5};

    // 使用 std::move 将 vec 的内容移动到 MyClass 的对象中，避免拷贝
    MyClass obj(std::move(vec));

    std::cout << "vec size after move: " << vec.size()
              << std::endl;  // vec 已经被清空

    return 0;
}
