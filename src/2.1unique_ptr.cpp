#include <iostream>
#include <memory>

// 手动实现 make_unique，用于 C++11 支持
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

class MyClass
{
public:
    MyClass() { std::cout << "MyClass constructor" << std::endl; }
    ~MyClass() { std::cout << "MyClass destructor" << std::endl; }
    void display() { std::cout << "Displaying MyClass object" << std::endl; }
};

int main()
{
    // 使用手动实现的 make_unique 创建 unique_ptr
    std::unique_ptr<MyClass> ptr1 = make_unique<MyClass>();
    ptr1->display();

    // 将 unique_ptr 传递给另一个 unique_ptr（所有权转移）
    std::unique_ptr<MyClass> ptr2 = std::move(ptr1);

    if (!ptr1)
    {
        std::cout << "ptr1 is now empty after move." << std::endl;
    }

    return 0;
}
