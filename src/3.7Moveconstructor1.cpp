#include <iostream>
#include <vector>

class MyClass
{
public:
    std::vector<int> data;

    // 默认构造函数
    MyClass() = default;  // 使用编译器生成的默认构造函数

    // 移动构造函数
    MyClass(MyClass&& other) noexcept : data(std::move(other.data))
    {
        std::cout << "Move constructor called!" << std::endl;
    }
};

int main()
{
    MyClass obj1;
    obj1.data = {1, 2, 3, 4, 5};

    MyClass obj2(std::move(obj1));  // 使用移动构造函数

    std::cout << "obj1 size after move: " << obj1.data.size()
              << std::endl;  // obj1的资源已被移动
    return 0;
}
