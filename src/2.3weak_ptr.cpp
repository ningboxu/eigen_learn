#include <iostream>
#include <memory>

class MyClass;

class B;  // 前向声明

class A {
public:
    std::shared_ptr<B> ptrB;  // A 拥有 B 的 shared_ptr
    ~A() {
        std::cout << "A is destroyed" << std::endl;
    }
};

class B {
public:
    std::weak_ptr<A> ptrA;  // B 拥有 A 的 weak_ptr，打破循环引用
    ~B() {
        std::cout << "B is destroyed" << std::endl;
    }
};

int main() {
    std::shared_ptr<A> a = std::make_shared<A>();
    std::shared_ptr<B> b = std::make_shared<B>();

    a->ptrB = b;  // A 持有 B 的 shared_ptr
    b->ptrA = a;  // B 持有 A 的 weak_ptr

    return 0;
}

