#include <iostream>
#include <memory>

class MyClass {
public:
    MyClass() {
        std::cout << "MyClass constructor" << std::endl;
    }
    ~MyClass() {
        std::cout << "MyClass destructor" << std::endl;
    }
    void display() {
        std::cout << "Displaying MyClass object" << std::endl;
    }
};

int main() {
    // 创建 shared_ptr
    std::shared_ptr<MyClass> ptr1 = std::make_shared<MyClass>();
    std::shared_ptr<MyClass> ptr2 = ptr1;  // 共享所有权

    std::cout << "Reference count: " << ptr1.use_count() << std::endl;  // 引用计数为 2

    ptr2->display();

    ptr1.reset();  // 重置 ptr1，不再共享对象

    std::cout << "Reference count after reset: " << ptr2.use_count() << std::endl;  // 引用计数为 1

    return 0;
}

